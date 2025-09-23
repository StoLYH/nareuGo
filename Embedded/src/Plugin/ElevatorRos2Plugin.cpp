// ElevatorRos2Plugin.cpp (ramped velocity control)
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <atomic>
#include <thread>
#include <algorithm>
#include <regex>   // 추가

namespace gazebo
{
    class ElevatorRos2Plugin : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
        {
            this->model_ = model;
            this->world_ = model->GetWorld();

            // joints
            jLift_  = model_->GetJoint("lift_joint");
            jDoorL_ = model_->GetJoint("door_left_joint");
            jDoorR_ = model_->GetJoint("door_right_joint");

            if (!jLift_)   gzerr << "[ElevatorRos2Plugin] lift_joint NOT FOUND\n";
            if (!jDoorL_)  gzerr << "[ElevatorRos2Plugin] door_left_joint NOT FOUND\n";
            if (!jDoorR_)  gzerr << "[ElevatorRos2Plugin] door_right_joint NOT FOUND\n";

            // 네임스페이스 읽기 (SDF 우선)
            std::string ns = "";
            if (sdf && sdf->HasElement("ros")) {
                auto rosElem = sdf->GetElement("ros");
                if (rosElem->HasElement("namespace")) {
                    ns = rosElem->Get<std::string>("namespace");
                }
            }

            // 네임스페이스 자동 생성: elevator, elevator_0.. → /elevator, /elevator2..
            if (ns.empty()) {
                const std::string model_name = model_->GetName();
                std::smatch m;
                if (model_name == "elevator") {
                    ns = "/elevator";
                } else if (std::regex_match(model_name, m, std::regex("^elevator_(\\d+)$"))) {
                    int idx = std::stoi(m[1].str()); // 0,1,2,..
                    ns = "/elevator" + std::to_string(idx + 2); // 0→2, 1→3, 2→4
                } else {
                    // 그 외 모델명은 기존 이름 그대로 사용
                    ns = "/" + model_name;
                }
            } else {
                if (!ns.empty() && ns.front() != '/') ns = "/" + ns;
            }

            // ROS2 init
            if (!rclcpp::ok())
            {
                int argc = 0; char **argv = nullptr;
                rclcpp::init(argc, argv);
            }

            // Node 생성: (노드이름, 네임스페이스, 옵션)
            rclcpp::NodeOptions opts;
            node_ = std::make_shared<rclcpp::Node>("elevator_ros2_plugin", ns, opts);
            RCLCPP_INFO(node_->get_logger(), "resolved namespace: %s", ns.c_str());

            // 토픽: 상대 이름 사용 -> 네임스페이스 자동 적용
            std::string topic_lift = "cmd_lift";
            std::string topic_door = "cmd_door";

            // subscribers from topics
            subLift_ = node_->create_subscription<std_msgs::msg::Float64>(
                topic_lift, 10,
                [this](std_msgs::msg::Float64::SharedPtr msg)
                {
                    RCLCPP_INFO(node_->get_logger(), "recv lift z=%.3f", msg->data);
                    target_z_ = msg->data; // just store target
                }
            );

            subDoor_ = node_->create_subscription<std_msgs::msg::Float64>(
                topic_door, 10,
                [this](std_msgs::msg::Float64::SharedPtr msg)
                {
                    RCLCPP_INFO(node_->get_logger(), "recv door value=%.3f", msg->data);
                    target_door_ = msg->data;
                }
            );

            // physics step callback: ramp velocities toward targets
            last_update_sim_time_ = world_->SimTime();            
            updateConn_ = event::Events::ConnectWorldUpdateBegin(
                [this](const gazebo::common::UpdateInfo &info)
                {
                    const auto now = info.simTime;
                    double dt = (now - last_update_sim_time_).Double();
                    if (dt <= 0) return;
                    last_update_sim_time_ = now;

                    // Lift control
                    if (jLift_)
                    {
                        double z = jLift_->Position(0); // prismatic position
                        double e = target_z_ - z;
                        double v_des = std::clamp(kp_pos_ * e, -vmax_lift_, vmax_lift_);
                        // accel limit
                        double dv = v_des - v_lift_;
                        double max_dv = amax_lift_ * dt; // amax_lift: 1.5
                        if (dv >  max_dv) dv =  max_dv;
                        if (dv < -max_dv) dv = -max_dv;
                        v_lift_ += dv;

                        // apply velocity to joint motor
                        jLift_->SetParam("fmax", 0, fmax_lift_);   // ensure motor effort available
                        jLift_->SetVelocity(0, v_lift_);          // prismatic: m/s
                    }

                    // // Door left
                    if (jDoorL_)
                    {
                        double q = jDoorL_->Position(0);
                        double e = target_door_ - q;
                        double v_des = std::clamp(kp_door_ * e, -vmax_door_, vmax_door_);
                        double dv = v_des - v_door_;
                        double max_dv = amax_door_ * dt;
                        if (dv >  max_dv) dv =  max_dv;
                        if (dv < -max_dv) dv = -max_dv;
                        v_door_ += dv;

                        jDoorL_->SetParam("fmax", 0, fmax_door_);
                        jDoorL_->SetVelocity(0, v_door_);       // revolute: rad/s
                    }

                    // // Door right
                    if (jDoorR_)
                    {
                        double q = jDoorR_->Position(0);
                        double e = target_door_ - q;
                        double v_des = std::clamp(kp_door_ * e, -vmax_door_, vmax_door_);
                        double dv = v_des - v_door_;
                        double max_dv = amax_door_ * dt;
                        if (dv >  max_dv) dv =  max_dv;
                        if (dv < -max_dv) dv = -max_dv;
                        v_door_ += dv;

                        jDoorR_->SetParam("fmax", 0, fmax_door_);
                        jDoorR_->SetVelocity(0, v_door_);
                    }
                });

            // spin 적용
            spinThread_ = std::thread([this]()
            {
                rclcpp::executors::SingleThreadedExecutor exec;
                exec.add_node(this->node_);
                exec.spin();
            });
        }

        ~ElevatorRos2Plugin() override
        {
            if (spinThread_.joinable())
            {
                exec_cancel_ = true;
                rclcpp::shutdown();
                spinThread_.join();
            }
        }

    private:
        physics::WorldPtr world_;
        physics::ModelPtr model_;
        physics::JointPtr jLift_, jDoorL_, jDoorR_;
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subLift_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subDoor_;
        event::ConnectionPtr updateConn_;
        std::thread spinThread_;
        std::atomic<bool> exec_cancel_{false};

        // targets
        double target_z_{0.0};
        double target_door_ {0.0};

        // simple P + rate limit params
        double kp_pos_{1.0};        // lift P gain [1/s]
        double vmax_lift_{0.6};     // m/s
        double amax_lift_{1.5};     // m/s^2
        double fmax_lift_{500.0};   // N (tune to mass/friction)

        double kp_door_{2.0};       // door P gain [1/s]
        double vmax_door_{1.2};     // rad/s
        double amax_door_{5.0};     // rad/s^2
        double fmax_door_{50.0};    // N·m

        // state
        gazebo::common::Time last_update_sim_time_;
        double v_lift_{0.0};
        double v_door_ {0.0};
    };

    GZ_REGISTER_MODEL_PLUGIN(ElevatorRos2Plugin)
} // namespace gazebo
