#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh> // Events 헤더 추가
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <thread> // thread 헤더 추가

namespace gazebo_plugins
{
class StorageDoorRos2Plugin : public gazebo::ModelPlugin
{
public:
  StorageDoorRos2Plugin() = default;
  ~StorageDoorRos2Plugin() = default;

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    this->model_ = _model;
    this->door_joint_ = this->model_->GetJoint("door_hinge_joint");

    if (!this->door_joint_) {
      RCLCPP_ERROR(rclcpp::get_logger("StorageDoorPlugin"), "door_hinge_joint not found!");
      return;
    }
    
    // 목표 각도를 현재 각도로 초기화
    this->target_angle_ = this->door_joint_->Position(0);

    // Gazebo의 월드 업데이트 이벤트에 OnUpdate 함수를 연결
    this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&StorageDoorRos2Plugin::OnUpdate, this));

    // ROS 2 노드 및 구독자 설정
    this->ros_node_ = rclcpp::Node::make_shared("storage_door_controller_plugin");
    this->subscription_ = this->ros_node_->create_subscription<std_msgs::msg::String>(
      "/door_control", 10,
      std::bind(&StorageDoorRos2Plugin::OnControlMsg, this, std::placeholders::_1));
    
    this->ros_thread_ = std::thread([this]() { rclcpp::spin(this->ros_node_); });

    RCLCPP_INFO(this->ros_node_->get_logger(), "Storage Door Plugin loaded successfully.");
  }

private:
  // ROS 메시지를 받으면 목표 각도만 설정
  void OnControlMsg(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "open") {
      this->target_angle_ = -1.83; // 목표 각도: 열림
      RCLCPP_INFO(this->ros_node_->get_logger(), "Command received: Open door.");
    } else if (msg->data == "close") {
      this->target_angle_ = 0.0;   // 목표 각도: 닫힘
      RCLCPP_INFO(this->ros_node_->get_logger(), "Command received: Close door.");
    }
  }

  // 매 시뮬레이션 스텝마다 호출되는 함수
  void OnUpdate()
  {
    double current_angle = this->door_joint_->Position(0);
    double error = this->target_angle_ - current_angle;
    
    // 목표 각도와 현재 각도의 오차에 비례하는 속도를 '지속적으로' 설정합니다.
    // 이렇게 하면 문이 목표 위치에서 벗어나려 할 때마다 제자리로 돌려놓는 힘이 작용하여
    // 주행 중에도 문이 고정됩니다.
    double velocity = error * 10.0; // 숫자를 키우면 더 강하게 고정됩니다 (P 제어기)

    this->door_joint_->SetVelocity(0, velocity);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointPtr door_joint_;
  
  // Gazebo 이벤트 및 목표 각도를 저장할 변수들 추가
  gazebo::event::ConnectionPtr update_connection_;
  double target_angle_;

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::thread ros_thread_;
};

GZ_REGISTER_MODEL_PLUGIN(StorageDoorRos2Plugin)
}
