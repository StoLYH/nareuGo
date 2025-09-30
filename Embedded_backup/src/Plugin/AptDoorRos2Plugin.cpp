#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <thread>
#include <map>
#include <vector>
#include <algorithm>
#include <cmath>

namespace gazebo
{
  struct JointCtrl {
    physics::JointPtr j;
    double v{0.0}; // 현재 명령 속도(램프용)
  };

  struct DoorPair {
    JointCtrl left;
    JointCtrl right;
  };

  class AptDoorRos2Plugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr model, sdf::ElementPtr) override
    {
      model_ = model;
      world_ = model_->GetWorld();

      floors_ = {"floor1","floor2","floor3","floor4"};
      for (auto& f : floors_)
      {
        DoorPair p;
        p.left.j  = model_->GetJoint(f + "_left_slide");
        p.right.j = model_->GetJoint(f + "_right_slide");
        if (!p.left.j || !p.right.j)
          gzerr << "[AptDoorRos2Plugin] joints missing for " << f << "\n";

        // ODE 파라미터: 리미트가 단단히 걸리도록
        for (auto* jp : {p.left.j.get(), p.right.j.get()})
        {
          if (!jp) continue;
          // 최대 힘
          jp->SetParam("fmax", 0, fmax_door_);
          // 리미트 정지 강성/감쇠
          jp->SetParam("stop_erp", 0, stop_erp_);
          jp->SetParam("stop_cfm", 0, stop_cfm_);
        }

        doors_[f]   = p;
        targets_[f] = 0.0; // 닫힘
        topics_.push_back("/apt_door/" + f + "/cmd_door");
      }

      // ROS2 init
      if (!rclcpp::ok()) { int argc=0; char**argv=nullptr; rclcpp::init(argc, argv); }
      node_ = std::make_shared<rclcpp::Node>("apt_door_ros2_plugin");

      // 구독
      for (size_t i = 0; i < topics_.size(); ++i)
      {
        const std::string floor = floors_[i];
        subs_.push_back(
          node_->create_subscription<std_msgs::msg::Float64>(
            topics_[i], 10,
            [this, floor](std_msgs::msg::Float64::SharedPtr msg)
            {
              // 명령 범위 클램프(-0.5~0.0)
              double v = std::max(-0.5, std::min(0.0, msg->data));
              targets_[floor] = v;
              RCLCPP_INFO(node_->get_logger(), "[%s] cmd=%.3f", floor.c_str(), v);
            })
        );
      }

      // 업데이트 루프
      updateConn_ = event::Events::ConnectWorldUpdateBegin(
        [this](const gazebo::common::UpdateInfo &info)
        {
          double dt = (info.simTime - lastSimTime_).Double();
          if (dt <= 0) return;
          lastSimTime_ = info.simTime;

          for (auto &kv : doors_)
          {
            const std::string& floor = kv.first;
            auto &pair   = kv.second;
            double target = targets_[floor];

            StepJoint(pair.left,  target, dt);
            StepJoint(pair.right, target, dt); // 축 방향 반전은 SDF에서 처리됨
          }
        });

      // ROS2 spin thread
      spinThread_ = std::thread([this]()
      {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(node_);
        exec.spin();
      });
    }

    ~AptDoorRos2Plugin() override
    {
      if (spinThread_.joinable()) {
        rclcpp::shutdown();
        spinThread_.join();
      }
    }

  private:
    inline double clamp(double v, double lo, double up) const {
      return std::max(lo, std::min(up, v));
    }

    void StepJoint(JointCtrl& jc, double target_cmd, double dt)
    {
      if (!jc.j) return;

      // 1) 실제 조인트 리미트로 타깃 클램프
      const double lo = jc.j->LowerLimit(0);
      const double up = jc.j->UpperLimit(0);
      double target = clamp(target_cmd, lo, up);

      const double cur = jc.j->Position(0);
      double err = target - cur;

      // 2) 목표 근접 시 정지 + 스냅
      if (std::abs(err) < eps_pos_) {
        jc.v = 0.0;
        jc.j->SetVelocity(0, 0.0);
        jc.j->SetPosition(0, target);
        return;
      }

      // 3) 비례 속도 → 최대 속도 제한
      double v_des = clamp(kp_door_ * err, -vmax_door_, vmax_door_);

      // 4) 가속도 램프(이전 속도에서 amax로만 변화)
      double dv_max = amax_door_ * dt;
      double v_cmd  = clamp(v_des, jc.v - dv_max, jc.v + dv_max);

      // 5) 한 스텝에 타깃을 넘지 않도록 과주행 방지
      double v_stop = err / dt; // 정확히 목표에 멈추는 속도
      if ((v_cmd > 0 && v_cmd > v_stop) || (v_cmd < 0 && v_cmd < v_stop)) {
        v_cmd = v_stop;
      }

      // 6) 리미트 근처에서 더 미는 명령 차단
      const double margin = 1e-4;
      if ((cur <= lo + margin && v_cmd < 0) ||
          (cur >= up - margin && v_cmd > 0)) {
        v_cmd = 0.0;
      }

      jc.v = v_cmd;
      jc.j->SetVelocity(0, v_cmd);
      // 최대 힘 보장
      jc.j->SetParam("fmax", 0, fmax_door_);
    }

  private:
    physics::WorldPtr world_;
    physics::ModelPtr model_;

    std::vector<std::string> floors_;
    std::map<std::string, DoorPair> doors_;
    std::map<std::string, double> targets_;
    std::vector<std::string> topics_;

    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> subs_;
    std::shared_ptr<rclcpp::Node> node_;
    event::ConnectionPtr updateConn_;
    std::thread spinThread_;
    gazebo::common::Time lastSimTime_;

    // 문 제어 파라미터(엘리베이터 방식)
    double kp_door_{6.0};     // 위치 비례 gain
    double vmax_door_{0.6};   // 최대 속도 [m/s]
    double amax_door_{2.0};   // 최대 가속도 [m/s^2]
    double fmax_door_{200.0}; // 조인트 최대 힘
    double eps_pos_{1e-3};    // 정지 임계치

    // ODE 리미트 정지 강성/감쇠(엘리베이터와 동일한 효과)
    double stop_erp_{0.2};
    double stop_cfm_{1e-6};
  };

  GZ_REGISTER_MODEL_PLUGIN(AptDoorRos2Plugin)
}