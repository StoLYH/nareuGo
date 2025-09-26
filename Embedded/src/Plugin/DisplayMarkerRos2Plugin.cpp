#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/msgs.hh>              // ignition::msgs::Marker (Gazebo11)
#include <ignition/math/Vector3.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>

namespace gazebo {

class DisplayMarkerRos2Plugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    std::cout << "[DisplayMarkerRos2Plugin] Loaded on model: " << model_->GetName() << std::endl;

    // Gazebo transport (~/marker)
    gz_node_ = transport::NodePtr(new transport::Node());
    gz_node_->Init(model_->GetWorld()->Name());
    marker_pub_ = gz_node_->Advertise<ignition::msgs::Marker>("~/marker");

    // SDF params
    if (sdf && sdf->HasElement("topic"))        topic_ = sdf->Get<std::string>("topic");
    if (sdf && sdf->HasElement("text_height"))  text_height_ = sdf->Get<double>("text_height");
    if (sdf && sdf->HasElement("front_offset")) front_offset_ = sdf->Get<double>("front_offset");

    // panels (dir: +1 => +Y, -1 => -Y)
    panels_.clear();
    if (sdf && sdf->HasElement("panels")) {
      auto panels = sdf->GetElement("panels");
      auto pe = panels->GetElement("panel");
      while (pe) {
        Panel p;
        p.link = pe->Get<std::string>("link");
        p.id   = pe->Get<int>("id");
        p.dir  = pe->HasAttribute("dir") ? pe->Get<int>("dir") : 1;
        panels_.push_back(p);
        pe = pe->GetNextElement("panel");
      }
    } else {
      panels_ = { {"panel_f1",1, 1}, {"panel_f2",2,-1}, {"panel_f3",3,-1}, {"panel_f4",4,-1} };
    }

    // ROS2 init / sub
    if (!rclcpp::ok()) { int argc=0; char** argv=nullptr; rclcpp::init(argc, argv); }
    ros_node_ = std::make_shared<rclcpp::Node>("display_marker_ros2_plugin");

    using std_msgs::msg::Float64;
    sub_ = ros_node_->create_subscription<Float64>(
      topic_, rclcpp::QoS(10),
      [this](Float64::SharedPtr msg){
        int f = MapHeightToFloor(msg->data);
        // 초기/0.0은 1층 처리
        if (!got_msg_.load() || msg->data == 0.0) f = 1;
        got_msg_.store(true);
        if (f != last_floor_) {
          last_floor_ = f;
          last_text_ = std::to_string(f);
          PublishAll(last_text_);
          std::cout << "[DisplayMarkerRos2Plugin] floor=" << last_text_ << std::endl;
        }
      });

    // ROS spin
    stop_.store(false);
    spin_thread_ = std::thread([this](){
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(ros_node_);
      while (rclcpp::ok() && !stop_.load()) {
        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    });

    // 초기 텍스트(초기 상태는 1층) + 1Hz 재발행
    last_text_ = "1";
    repub_thread_ = std::thread([this](){
      using namespace std::chrono_literals;
      while (!stop_.load()) {
        PublishAll(last_text_);
        std::this_thread::sleep_for(1s);
      }
    });
  }

  ~DisplayMarkerRos2Plugin() override {
    // 마커 삭제
    for (const auto &p : panels_) {
      ignition::msgs::Marker m;
      m.set_ns(ns_);
      m.set_id(p.id);
      m.set_action(ignition::msgs::Marker::DELETE_MARKER);
      marker_pub_->Publish(m);
    }
    stop_.store(true);
    if (repub_thread_.joinable()) repub_thread_.join();
    if (spin_thread_.joinable())  spin_thread_.join();
  }

private:
  struct Panel { std::string link; int id; int dir{1}; };

  // 높이→층수 매핑: 0.0→1, 1.05→2, 2.05→3, 3.10→4 (±0.03 허용)
  int MapHeightToFloor(double z) const {
    const double tol = 0.03;
    if (std::fabs(z - 3.10) < tol) return 4;
    if (std::fabs(z - 2.05) < tol) return 3;
    if (std::fabs(z - 1.05) < tol) return 2;
    // 나머지는 1층 취급(초기/0 포함)
    return 1;
  }

  void PublishAll(const std::string &text) {
    for (const auto &p : panels_) PublishOne(p, text);
  }

  void PublishOne(const Panel &p, const std::string &text) {
    auto link = model_->GetLink(p.link);
    if (!link) return;

    const auto wp  = link->WorldPose();
    const auto dir = wp.Rot().RotateVector(ignition::math::Vector3d(0, p.dir, 0));
    const auto pos = wp.Pos() + dir * front_offset_;

    ignition::msgs::Marker m;
    m.set_ns(ns_);
    m.set_id(p.id);
    m.set_action(ignition::msgs::Marker::ADD_MODIFY);
    m.set_type(ignition::msgs::Marker::TEXT);

    auto pose = m.mutable_pose();
    pose->mutable_position()->set_x(pos.X());
    pose->mutable_position()->set_y(pos.Y());
    pose->mutable_position()->set_z(pos.Z());
    pose->mutable_orientation()->set_w(wp.Rot().W());
    pose->mutable_orientation()->set_x(wp.Rot().X());
    pose->mutable_orientation()->set_y(wp.Rot().Y());
    pose->mutable_orientation()->set_z(wp.Rot().Z());

    m.mutable_scale()->set_x(text_height_); // 문자 높이
    m.mutable_scale()->set_y(0.0);
    m.mutable_scale()->set_z(0.0);

    auto mat = m.mutable_material();
    mat->mutable_ambient()->set_r(0.95); mat->mutable_ambient()->set_g(0.85);
    mat->mutable_ambient()->set_b(0.10); mat->mutable_ambient()->set_a(1.0);
    mat->mutable_diffuse()->set_r(0.95); mat->mutable_diffuse()->set_g(0.85);
    mat->mutable_diffuse()->set_b(0.10); mat->mutable_diffuse()->set_a(1.0);
    mat->mutable_emissive()->set_r(1.0); mat->mutable_emissive()->set_g(0.9);
    mat->mutable_emissive()->set_b(0.2); mat->mutable_emissive()->set_a(1.0);

    m.set_text(text);
    marker_pub_->Publish(m);
  }

private:
  physics::ModelPtr model_;
  transport::NodePtr gz_node_;
  transport::PublisherPtr marker_pub_;

  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
  std::thread spin_thread_, repub_thread_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> got_msg_{false};

  std::vector<Panel> panels_;
  std::string topic_{"/elevator/cmd_lift"};
  std::string ns_{"elevator_display"};
  double text_height_{0.22};
  double front_offset_{0.010};
  int    last_floor_{1};
  std::string last_text_{"1"};
};

GZ_REGISTER_MODEL_PLUGIN(DisplayMarkerRos2Plugin)

} // namespace gazebo