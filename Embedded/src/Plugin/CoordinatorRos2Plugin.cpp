#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>

#include <thread>
#include <atomic>
#include <string>
#include <map>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>              
#include <sstream>             
#include <gazebo_ros/node.hpp> 

using namespace std;


namespace gazebo
{

    class CoordinatorRos2Plugin : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
        {
            // gazebo_ros 노드 사용
            ros_node_ = gazebo_ros::Node::Get(sdf);

            model_ = model;
            world_ = model_ ? model_->GetWorld() : nullptr;

            // --- 추가: SDF 파라미터로 네임스페이스/도어 프리픽스 읽기 ---
            // 기본값: 기존 코드와 동일
            elevator_ns_ = "elevator";
            door_prefix_ = "apt_door";
            if (sdf) {
                if (sdf->HasElement("elevator_ns")) {
                    elevator_ns_ = sdf->Get<std::string>("elevator_ns");
                } else {
                    // 모델명으로 간단 추정(옵션): 필요 없으면 삭제 가능
                    const auto& mn = model_ ? model_->GetName() : std::string();
                    if (mn.find("elevator_2") != std::string::npos || mn.find("apartment4") != std::string::npos)
                        elevator_ns_ = "elevator_2";
                    else if (mn.find("elevator_0") != std::string::npos || mn.find("apartment2") != std::string::npos)
                        elevator_ns_ = "elevator_0";
                }
                if (sdf->HasElement("door_prefix")) {
                    door_prefix_ = sdf->Get<std::string>("door_prefix");
                }
            }
            RCLCPP_INFO(ros_node_->get_logger(), "[Coordinator] using ns='%s', door_prefix='%s'",
                        elevator_ns_.c_str(), door_prefix_.c_str());

            // elevator 모델에서 조인트 가져오기
            jLift_ = model_->GetJoint("lift_joint");
            jDoorL_ = model_->GetJoint("door_left_joint");
            jDoorR_ = model_->GetJoint("door_right_joint");

            // 퍼블리셔: 엘리베이터별 절대 토픽 (cmd_*로 변경)
            pubLift_  = ros_node_->create_publisher<std_msgs::msg::Float64>("/" + elevator_ns_ + "/cmd_lift", 10);
            pubEDoor_ = ros_node_->create_publisher<std_msgs::msg::Float64>("/" + elevator_ns_ + "/cmd_door", 10);

            // 상태 토픽은 엘리베이터 네임스페이스 하위(그대로)
            pubReady_   = ros_node_->create_publisher<std_msgs::msg::Bool>("/" + elevator_ns_ + "/status/ready", 10);
            pubArrived_ = ros_node_->create_publisher<std_msgs::msg::Int32>("/" + elevator_ns_ + "/status/arrived", 10);
            pubState_   = ros_node_->create_publisher<std_msgs::msg::String>("/" + elevator_ns_ + "/status/state", 10);

            // 층문 퍼블리셔: 모든 아파트 공통 절대 토픽
            for (int f = 1; f <= 4; ++f) {
                const std::string fdoor_topic = "/" + door_prefix_ + "/floor" + std::to_string(f) + "/cmd_door";
                pubFDoor_[f] = ros_node_->create_publisher<std_msgs::msg::Float64>(fdoor_topic, 10);
            }

            // 구독자: 엘리베이터 네임스페이스 하위(상대 토픽 유지)
            subCurFloor_ = ros_node_->create_subscription<std_msgs::msg::Int32>(
                "robot/current_location", rclcpp::QoS(10),
                [this](std_msgs::msg::Int32::SharedPtr m)
                {
                    if (m->data >= 1 && m->data <= 4)
                    {
                        curFloorFromRobot_ = m->data;
                        gotCurrentFloor_ = true;
                    }
                });

            subTargetFloor_ = ros_node_->create_subscription<std_msgs::msg::Int32>(
                "robot/target_floor", rclcpp::QoS(10),
                [this](std_msgs::msg::Int32::SharedPtr m)
                {
                    if (m->data >= 1 && m->data <= 4)
                    {
                        targetFloorFromRobot_ = m->data;
                        gotTargetFloor_ = true;
                    }
                });

            subBoarded_ = ros_node_->create_subscription<std_msgs::msg::Bool>(
                "robot/boarded", rclcpp::QoS(10),
                [this](std_msgs::msg::Bool::SharedPtr m)
                { boarded_ = m->data; });

            subAlighted_ = ros_node_->create_subscription<std_msgs::msg::Bool>(
                "robot/alighted", rclcpp::QoS(10),
                [this](std_msgs::msg::Bool::SharedPtr m)
                { alighted_ = m->data; });

            // 조인트 로그
            if (model_)
            {
                std::ostringstream oss;
                oss << "[Coordinator] joints:";
                for (auto &j : model_->GetJoints())
                    oss << " " << j->GetName();
                RCLCPP_INFO(ros_node_->get_logger(), "%s", oss.str().c_str());
            }
            if (!jLift_)
                RCLCPP_ERROR(ros_node_->get_logger(), "[Coordinator] lift_joint not found");
            if (!jDoorL_)
                RCLCPP_WARN(ros_node_->get_logger(), "[Coordinator] door_left_joint not found");
            if (!jDoorR_)
                RCLCPP_WARN(ros_node_->get_logger(), "[Coordinator] door_right_joint not found");

            // 층-높이 매핑(1-index)
            floorZ_.clear();
            floorZ_.resize(1);
            floorZ_.push_back(0.00); // 1층
            floorZ_.push_back(1.072); // 2층
            floorZ_.push_back(2.072); // 3층
            floorZ_.push_back(3.072); // 4층

            // 절대 높이 캘리브레이션: absZ = liftZeroZ_ + Position(0)
            if (jLift_)
            {
                liftChild_ = jLift_->GetChild();
                const double childZ = (liftChild_) ? liftChild_->WorldPose().Pos().Z() : 0.0;
                const double jpos = jLift_->Position(0);
                liftZeroZ_ = childZ - jpos;
                liftCalibrated_ = (liftChild_ != nullptr);
                RCLCPP_INFO(ros_node_->get_logger(),
                            "[Coordinator] lift calibration: child=%s childZ=%.3f jpos=%.3f zeroZ=%.3f",
                            liftChild_ ? liftChild_->GetName().c_str() : "(null)", childZ, jpos, liftZeroZ_);
            }

            // Gazebo 업데이트 루프 연결
            if (world_) {
                lastSim_ = world_->SimTime();
                updateConn_ = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&CoordinatorRos2Plugin::OnUpdate, this));
                // 초기 상태 발행
                SendState();
            } else {
                RCLCPP_ERROR(ros_node_->get_logger(), "[Coordinator] world is null");
            }
        }

        ~CoordinatorRos2Plugin() override
        {
            if (spinThread_.joinable())
            {
                rclcpp::shutdown();
                spinThread_.join();
            }
        }

    private:
        enum class State
        {
            WAIT_CALL,
            MOVE_TO_PICKUP,
            OPEN_PICKUP_DOORS,
            WAIT_BOARD,
            CLOSE_PICKUP_DOORS,
            MOVE_TO_DEST,
            OPEN_DEST_DOORS,
            WAIT_ALIGHT,
            CLOSE_DEST_DOORS,
            IDLE
        };

        // 매 틱마다 호출되어 Step(dt) 실행
        void OnUpdate()
        {
            if (!world_) return;
            auto now = world_->SimTime();
            double dt = (now - lastSim_).Double();
            if (dt < 0.0) dt = 0.0;
            if (dt > 0.1) dt = 0.1; // 폭주 방지
            lastSim_ = now;
            Step(dt);
        }

        // 오차(도착 판정용)
        double zTol_{0.03};

        // 실제 층 개수 반환 (FloorZ.size()가 더미 1칸을 포함하므로 -1)
        size_t FloorsCount() const { 
            return (floorZ_.size() > 0) ? floorZ_.size() - 1 : 0; 
        }
        // 유효한 층 번호인지 확인 후 층 개수 반환 (f가 유효한 층 번호인지 검사)
        bool IsValidFloor(int f) const { 
            return f >= 1 && f <= static_cast<int>(FloorsCount()); 
        }

        // 층 번호를 높이로 변환
        double FloorToZ(int f) const
        {
            if (IsValidFloor(f))
                cout << f << "층 높이: " << floorZ_[static_cast<size_t>(f)] << endl;
                return floorZ_[static_cast<size_t>(f)];
            // 유효하지 않으면 1층 높이로 폴백
            return FloorsCount() >= 1 ? floorZ_[1] : 0.0;
        }

        // 높이를 가장 가까운 층 번호로 변환
        int ZToFloor(double z) const
        {
            if (FloorsCount() == 0)
                return 1; // 층 정보 없으면 1 반환
            int bestF = 1;
            double bestD = std::numeric_limits<double>::infinity();
            for (int f = 1; f <= static_cast<int>(FloorsCount()); ++f)
            {
                double d = std::fabs(z - floorZ_[static_cast<size_t>(f)]);
                if (d < bestD)
                {
                    bestD = d;
                    bestF = f;
                }
            }
            // 공차 내면 해당 층, 아니면 가장 가까운 층
            return bestF;
        }

        // 엘리베이터 리프트 조인트 위치
        double JL() const
        {
            if (jLift_)
            {
                // 절대 위치 보정
                if (liftCalibrated_)
                {
                    return liftZeroZ_ + jLift_->Position(0);
                }
                // 보정 안 됐으면 상대 위치
                if (auto child = jLift_->GetChild())
                {
                    return child->WorldPose().Pos().Z();
                }
            }
            return 0.0;
        }

        // 엘리베이터 문(좌/우) 조인트 위치 평균
        double JDavg() const
        {
            double l = jDoorL_ ? jDoorL_->Position(0) : 0.0;
            double r = jDoorR_ ? jDoorR_->Position(0) : 0.0;
            return 0.5 * (l + r);
        }

        // 엘리베이터가 f층에 도착했는지 판정
        bool IsLiftAtFloor(int f) const
        {
            if (f < 1 || f >= static_cast<int>(floorZ_.size()))
                return false;
            if (!jLift_)
                return false;
            return std::fabs(JL() - floorZ_[static_cast<size_t>(f)]) < zTol_;
        }

        // 엘리베이터 문이 열렸는지/닫혔는지 판정
        bool DoorsOpened() const { return JDavg() < -0.45; }
        bool DoorsClosed() const { return JDavg() > -0.02; }

        // lift publish
        void CmdLiftToFloor(int f)
        {
            std_msgs::msg::Float64 m;
            m.data = FloorToZ(f);
            cout << "CmdLiftToFloor: " << f << "층 (z=" << m.data << ")" << endl;
            cout << "  현재 JL=" << JL() << endl;
            pubLift_->publish(m);
        }

        // door publish
        void CmdAllDoorsOpenAtFloor(int f)
        {
            std_msgs::msg::Float64 open;
            open.data = -0.5;
            pubEDoor_->publish(open);
            auto it = pubFDoor_.find(f);
            if (it != pubFDoor_.end())
                it->second->publish(open);
        }
        void CmdAllDoorsCloseAtFloor(int f)
        {
            std_msgs::msg::Float64 close;
            close.data = 0.0;
            pubEDoor_->publish(close);
            auto it = pubFDoor_.find(f);
            if (it != pubFDoor_.end())
                it->second->publish(close);
        }

        // ready publish
        void PublishReady(bool v)
        {
            std_msgs::msg::Bool b;
            b.data = v;
            pubReady_->publish(b);
        }
        // arrived publish
        void PublishArrived(int f)
        {
            std_msgs::msg::Int32 m;
            m.data = f;
            pubArrived_->publish(m);
        }
        // state publish
        void SendState()
        {
            std_msgs::msg::String s;
            s.data = StateName(state_);
            pubState_->publish(s);
        }
        std::string StateName(State s) const
        {
            switch (s)
            {
            case State::WAIT_CALL:
                return "WAIT_CALL";
            case State::MOVE_TO_PICKUP:
                return "MOVE_TO_PICKUP";
            case State::OPEN_PICKUP_DOORS:
                return "OPEN_PICKUP_DOORS";
            case State::WAIT_BOARD:
                return "WAIT_BOARD";
            case State::CLOSE_PICKUP_DOORS:
                return "CLOSE_PICKUP_DOORS";
            case State::MOVE_TO_DEST:
                return "MOVE_TO_DEST";
            case State::OPEN_DEST_DOORS:
                return "OPEN_DEST_DOORS";
            case State::WAIT_ALIGHT:
                return "WAIT_ALIGHT";
            case State::CLOSE_DEST_DOORS:
                return "CLOSE_DEST_DOORS";
            case State::IDLE:
                return "IDLE";
            }
            return "UNKNOWN";
        }

        // 상태 전환되면 타이머/플래그 리셋
        void Transit(State s)
        {
            state_ = s;
            tInState_ = 0.0;         // 새 상태로 들어온 시간을 0으로 리셋
            doorCmdSent_ = false;    // 새 상태에서 문 열기/닫기 한 번만
            liftCmdLatched_ = false; // 새 상태에서 리프트 한 번만
            
            // WAIT_ALIGHT로 진입할 때 이전 하차 신호 잔존 방지
            if (s == State::WAIT_ALIGHT) {
                alighted_ = false;  // 새 하차 신호만 인정
            }

            SendState();             // 상태 변경 시 즉시 발행
        }

        // 매 시뮬레이션 스텝마다 호출
        void Step(double dt)
        {
            tInState_ += dt;

            auto curZ = JL();
            // 로그 출력용
            RCLCPP_INFO_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 10000,
                                 "state=%s JL=%.3f pickup=%d target=%d",
                                 StateName(state_).c_str(), curZ, pickupFloor_, targetFloorFromRobot_);

            switch (state_)
            {
            // WAIT_CALL: 로봇이 현재층 알려줄 때까지 대기
            case State::WAIT_CALL:
            {
                if (gotCurrentFloor_)
                {
                    pickupFloor_ = curFloorFromRobot_; // 로봇 위치
                    cout << "로봇 현재 위치: " << pickupFloor_ << "층" << endl;
                    gotCurrentFloor_ = false;          // 한 번만 처리
                    // 층이 다르면, lift publish
                    if (!IsLiftAtFloor(pickupFloor_))
                    {
                        CmdLiftToFloor(pickupFloor_);
                        liftCmdLatched_ = true;
                        Transit(State::MOVE_TO_PICKUP);
                    }
                    else
                    {
                        Transit(State::OPEN_PICKUP_DOORS);
                    }
                }
                break;
            }
            // MOVE_TO_PICKUP: 엘베를 로봇 있는 곳으로 이동 + 열림
            case State::MOVE_TO_PICKUP:
            {
                // 처음 1회 + 0.5초마다 보강 발행
                if (!liftCmdLatched_ || std::fmod(tInState_, 0.5) < 1e-3)
                {
                    CmdLiftToFloor(pickupFloor_);
                    liftCmdLatched_ = true;
                }
                if (IsLiftAtFloor(pickupFloor_) && tInState_ > 0.2)
                {
                    Transit(State::OPEN_PICKUP_DOORS);
                }
                break;
            }
            // OPEN_PICKUP_DOORS: 도착 후 문 열기
            case State::OPEN_PICKUP_DOORS:
            {
                if (!IsLiftAtFloor(pickupFloor_)) { Transit(State::MOVE_TO_PICKUP); break; }
                if (!doorCmdSent_) { CmdAllDoorsOpenAtFloor(pickupFloor_); doorCmdSent_ = true; }
                if (DoorsOpened()) { Transit(State::WAIT_BOARD); } // 완전 개방 시점에만
                break;
            }
            // WAIT_BOARD: 탑승 완료 + 목적층 수신까지 대기
            case State::WAIT_BOARD:
            {
                // 탑승 완료 + 목적층 수신까지 대기
                if (boarded_ && gotTargetFloor_)
                {
                    // 문 닫기 단계로
                    PublishReady(false);
                    Transit(State::CLOSE_PICKUP_DOORS);
                }
                // 안전을 위해 계속 문 열림 유지
                CmdAllDoorsOpenAtFloor(pickupFloor_);
                break;
            }
            // CLOSE_PICKUP_DOORS: 탑승 완료 후 문 닫기
            case State::CLOSE_PICKUP_DOORS:
            {
                CmdAllDoorsCloseAtFloor(pickupFloor_);
                if (DoorsClosed() || tInState_ > 1.5)
                {
                    // 이동 시작
                    CmdLiftToFloor(targetFloorFromRobot_);
                    Transit(State::MOVE_TO_DEST);
                }
                break;
            }
            // MOVE_TO_DEST: 엘베를 목적층으로 이동
            case State::MOVE_TO_DEST:
            {
                CmdLiftToFloor(targetFloorFromRobot_);
                if (IsLiftAtFloor(targetFloorFromRobot_) && tInState_ > 0.2)
                {
                    PublishArrived(targetFloorFromRobot_);
                    Transit(State::OPEN_DEST_DOORS);
                }
                break;
            }
            // OPEN_DEST_DOORS: 목적층 도착 후 문 열기
            case State::OPEN_DEST_DOORS:
            {
                CmdAllDoorsOpenAtFloor(targetFloorFromRobot_);
                if (DoorsOpened() || tInState_ > 1.5)
                {
                    Transit(State::WAIT_ALIGHT);
                }
                break;
            }
            // WAIT_ALIGHT: 하차 완료 대기
            case State::WAIT_ALIGHT:
            {
                // 로봇 하차 완료 대기
                if (alighted_)
                {
                    Transit(State::CLOSE_DEST_DOORS);
                }
                // 하차 동안 계속 문 열림 유지
                CmdAllDoorsOpenAtFloor(targetFloorFromRobot_);
                break;
            }
            // CLOSE_DEST_DOORS: 하차 완료 후 문 닫기
            case State::CLOSE_DEST_DOORS:
            {
                CmdAllDoorsCloseAtFloor(targetFloorFromRobot_);
                if (DoorsClosed() || tInState_ > 1.5)
                {
                    // 초기화 후 대기
                    ResetSession();
                    Transit(State::IDLE);
                }
                break;
            }
            // IDLE: 다음 호출을 위해 리셋하고 대기
            case State::IDLE:
            {
                // 다음 호출을 위해 리셋하고 대기
                if (gotCurrentFloor_)
                    Transit(State::WAIT_CALL);
                break;
            }
            }
        }

        void ResetSession()
        {
            gotCurrentFloor_ = false;
            boarded_ = false;
            gotTargetFloor_ = false;
            alighted_ = false;
        }

    private:
        // Gazebo
        physics::WorldPtr world_;
        physics::ModelPtr model_;
        physics::JointPtr jLift_, jDoorL_, jDoorR_;
        event::ConnectionPtr updateConn_;
        gazebo::common::Time lastSim_;

        // ROS2
        gazebo_ros::Node::SharedPtr ros_node_; // 추가
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subCurFloor_, subTargetFloor_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subBoarded_, subAlighted_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubLift_, pubEDoor_;
        std::map<int, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> pubFDoor_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubReady_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pubArrived_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubState_;
        std::thread spinThread_;
        State state_{State::WAIT_CALL};
        double tInState_{0.0};
        int pickupFloor_{1};
        int targetFloorFromRobot_{1};
        int curFloorFromRobot_{1};
        bool gotCurrentFloor_{false};
        bool boarded_{false};
        bool gotTargetFloor_{false};
        bool alighted_{false};
        std::vector<double> floorZ_;

        // 1회 발행 제어
        bool doorCmdSent_{false};
        bool liftCmdLatched_{false};

        // Lift calibration
        physics::LinkPtr liftChild_;
        double liftZeroZ_{0.0};
        bool liftCalibrated_{false};

        // 네임스페이스/프리픽스
        std::string elevator_ns_{"elevator"};
        std::string door_prefix_{"apt_door"};
    };

    GZ_REGISTER_MODEL_PLUGIN(CoordinatorRos2Plugin)

} // namespace gazebo