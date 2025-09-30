#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <thread>
#include <mutex>

namespace gazebo
{
class BoxControlPlugin : public WorldPlugin
{
public:
  BoxControlPlugin() : box_spawned_(false) {}
  ~BoxControlPlugin() = default;

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
  {
    this->world_ = _world;
    this->gz_node_ = transport::NodePtr(new transport::Node());
    this->gz_node_->Init();
    this->factory_pub_ = this->gz_node_->Advertise<msgs::Factory>("~/factory");
    this->request_pub_ = this->gz_node_->Advertise<msgs::Request>("~/request");

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&BoxControlPlugin::OnUpdate, this));

    this->ros_node_ = rclcpp::Node::make_shared("box_control_plugin");
    this->subscription_ = this->ros_node_->create_subscription<std_msgs::msg::String>(
      "/box_control", 10,
      std::bind(&BoxControlPlugin::OnControlMsg, this, std::placeholders::_1));

    this->ros_thread_ = std::thread([this]() { rclcpp::spin(this->ros_node_); });

    RCLCPP_INFO(this->ros_node_->get_logger(), "Box Control Plugin (Final Version) loaded.");
  }

private:
  void OnControlMsg(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "spawn") {
      SpawnBox();
    } else if (msg->data == "delete") {
      DeleteBox();
    }
  }

  void OnUpdate()
  {
    // 상자가 스폰된 상태일 때만 위치를 추적하고 업데이트합니다.
    if (this->box_spawned_) {
      physics::ModelPtr robot_model = this->world_->ModelByName("turtlebot3");
      physics::ModelPtr box_model = this->world_->ModelByName("small_red_box");

      // 로봇과 상자 모델이 모두 존재할 때만 위치를 업데이트합니다.
      if (robot_model && box_model) {
        ignition::math::Pose3d robot_pose = robot_model->WorldPose();
        ignition::math::Vector3d local_offset(-0.068, 0.0, 0.145);
        ignition::math::Vector3d world_offset = robot_pose.Rot() * local_offset;
        ignition::math::Vector3d box_position = robot_pose.Pos() + world_offset;
        ignition::math::Quaterniond box_rotation = robot_pose.Rot();
        ignition::math::Pose3d box_world_pose(box_position, box_rotation);

        box_model->SetWorldPose(box_world_pose);
      }
    }
  }

  void SpawnBox()
  {
    // 상자가 이미 스폰되었으면 다시 스폰하지 않습니다.
    if (this->box_spawned_ || this->world_->ModelByName("small_red_box")) {
        RCLCPP_WARN(this->ros_node_->get_logger(), "Box already exists. Spawn command ignored.");
        return;
    }

    // 초기 위치 계산을 위해 로봇 모델을 찾습니다.
    physics::ModelPtr robot_model = this->world_->ModelByName("turtlebot3");
    if (!robot_model) {
        RCLCPP_ERROR(this->ros_node_->get_logger(), "Robot 'turtlebot3' not found. Cannot calculate initial spawn position.");
        return;
    }

    // 초기 위치를 계산합니다.
    ignition::math::Pose3d robot_pose = robot_model->WorldPose();
    ignition::math::Vector3d local_offset(-0.068, 0.0, 0.145);
    ignition::math::Vector3d world_offset = robot_pose.Rot() * local_offset;
    ignition::math::Vector3d box_position = robot_pose.Pos() + world_offset;
    ignition::math::Quaterniond box_rotation = robot_pose.Rot();
    ignition::math::Pose3d initial_pose(box_position, box_rotation);

    // Factory 메시지를 통해 모델을 스폰합니다.
    msgs::Factory factory_msg;
    factory_msg.set_sdf_filename("model://small_red_box");
    msgs::Set(factory_msg.mutable_pose(), initial_pose);
    this->factory_pub_->Publish(factory_msg);

    this->box_spawned_ = true; // 스폰 상태 플래그를 true로 설정
    RCLCPP_INFO(this->ros_node_->get_logger(), "Spawn command sent. Box tracking enabled.");
  }

  void DeleteBox()
  {
    msgs::Request *request = msgs::CreateRequest("entity_delete", "small_red_box");
    this->request_pub_->Publish(*request);
    delete request;

    this->box_spawned_ = false; // 스폰 상태 플래그를 false로 설정
    RCLCPP_INFO(this->ros_node_->get_logger(), "Delete command sent. Box tracking disabled.");
  }

  physics::WorldPtr world_;
  transport::NodePtr gz_node_;
  transport::PublisherPtr factory_pub_;
  transport::PublisherPtr request_pub_;

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::thread ros_thread_;

  event::ConnectionPtr update_connection_;
  bool box_spawned_; // 상자가 스폰되었는지 여부를 추적하는 플래그
};

GZ_REGISTER_WORLD_PLUGIN(BoxControlPlugin)
}