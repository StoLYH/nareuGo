#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <array>
#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <chrono>
#include <cctype>
#include <cmath>

namespace gazebo
{
  class PanelDisplayVisualPlugin : public VisualPlugin {
  public:
    PanelDisplayVisualPlugin() = default;
    ~PanelDisplayVisualPlugin() override
    {
      // 렌더 콜백 해제
      this->updateConn_.reset();

      // ROS 스핀 종료 (전역 shutdown 사용하지 않음)
      stop_.store(true);
      if (rosSpin_.joinable())
        rosSpin_.join();

      node_.reset();
    }

    void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf) override
    {
      this->visual_ = _parent;

      // Gazebo transport publisher (~/visual)
      this->gzNode_ = transport::NodePtr(new transport::Node());
      this->gzNode_->Init(this->visual_->GetScene()->Name());
      this->visPub_ = this->gzNode_->Advertise<msgs::Visual>("~/visual");

      // ROS2 init
      if (!rclcpp::ok())
      {
        int argc = 0; char **argv = nullptr;
        rclcpp::init(argc, argv);
      }

      // 각 패널마다 고유한 노드 이름 생성
      auto sanitize = [](std::string s){
        for (auto &ch : s) if (!std::isalnum(static_cast<unsigned char>(ch)) && ch!='_') ch = '_';
        if (s.empty()) s = "display";
        return s;
      };
      std::string node_name = "panel_display_visual_" + sanitize(this->visual_->Name());
      node_ = std::make_shared<rclcpp::Node>(node_name);

      // 기본 구독 토픽: /elevator/cmd_lift
      std::string topic = "/elevator/cmd_lift";
      if (_sdf && _sdf->HasElement("topic"))
        topic = _sdf->Get<std::string>("topic");

      // 표시 오프셋 및 임계값 파라미터
      if (_sdf && _sdf->HasElement("offset_y"))
        frontOffsetY_ = _sdf->Get<double>("offset_y");
      if (_sdf && _sdf->HasElement("h2")) h2_ = _sdf->Get<double>("h2");
      if (_sdf && _sdf->HasElement("h3")) h3_ = _sdf->Get<double>("h3");
      if (_sdf && _sdf->HasElement("h4")) h4_ = _sdf->Get<double>("h4");
      if (_sdf && _sdf->HasElement("tol")) tol_ = _sdf->Get<double>("tol");

      using std_msgs::msg::Float64;
      this->sub_ = node_->create_subscription<Float64>(
        topic, rclcpp::QoS(10),
        [this](Float64::SharedPtr msg)
        {
          const double z = msg->data;
          int floor = this->MapHeightToFloor(z);
          this->desiredDigit_.store(floor);
        });

      // ROS 스핀 스레드 (shutdown에 의존하지 않고 플래그로 종료)
      rosSpin_ = std::thread([this]()
      {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(this->node_);
        while (rclcpp::ok() && !this->stop_.load())
        {
          exec.spin_some();
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
      });

      // 세그먼트 생성
      this->CreateSegments();

      // 렌더 업데이트 연결
      this->updateConn_ = event::Events::ConnectPreRender(
        std::bind(&PanelDisplayVisualPlugin::OnPreRender, this));
    }

  private:
    // 높이 → 층수 매핑 (허용 오차 사용)
    int MapHeightToFloor(double z)
    {
      auto nearEq = [this](double a, double b){ return std::abs(a - b) < tol_; };
      if (nearEq(z, h4_)) return 4;
      if (nearEq(z, h3_)) return 3;
      if (nearEq(z, h2_)) return 2;
      return 1; // 그 외는 1
    }

    void OnPreRender()
    {
      const int want = this->desiredDigit_.load();
      if (want == this->currentDigit_)
        return;

      this->ApplyDigit(want);
      this->currentDigit_ = want;
    }

    // 세그먼트 생성: 패널 전면으로 돌출시키기 위해 offset_y 사용
    void CreateSegments()
    {
      const double yFront = frontOffsetY_;
      this->SpawnBoxSeg("a",  0.000, yFront,  0.070,  0.090, 0.004, 0.020);
      this->SpawnBoxSeg("b",  0.045, yFront,  0.035,  0.020, 0.004, 0.120);
      this->SpawnBoxSeg("c",  0.045, yFront, -0.035,  0.020, 0.004, 0.120);
      this->SpawnBoxSeg("d",  0.000, yFront, -0.070,  0.090, 0.004, 0.020);
      this->SpawnBoxSeg("e", -0.045, yFront, -0.035,  0.020, 0.004, 0.120);
      this->SpawnBoxSeg("f", -0.045, yFront,  0.035,  0.020, 0.004, 0.120);
      this->SpawnBoxSeg("g",  0.000, yFront,  0.000,  0.090, 0.004, 0.020);

      // 초기 표시(1층)
      this->ApplyDigit(1);
      this->currentDigit_ = 1;
    }

    void SpawnBoxSeg(const std::string &segName,
                     double px, double py, double pz,
                     double sx, double sy, double sz)
    {
      msgs::Visual vmsg;
      const std::string fullName = this->visual_->Name() + "/seg_" + segName;

      vmsg.set_name(fullName);
      vmsg.set_parent_name(this->visual_->Name());
      vmsg.set_is_static(true);

      // Pose 직접 설정 (ignition::math 미사용)
      auto pose = vmsg.mutable_pose();
      pose->mutable_position()->set_x(px);
      pose->mutable_position()->set_y(py);
      pose->mutable_position()->set_z(pz);
      pose->mutable_orientation()->set_w(1.0);
      pose->mutable_orientation()->set_x(0.0);
      pose->mutable_orientation()->set_y(0.0);
      pose->mutable_orientation()->set_z(0.0);

      // Box geometry
      auto geom = vmsg.mutable_geometry();
      geom->set_type(msgs::Geometry::BOX);
      auto box = geom->mutable_box();
      box->mutable_size()->set_x(sx);
      box->mutable_size()->set_y(sy);
      box->mutable_size()->set_z(sz);

      // 기본 OFF
      this->SetMaterial(vmsg.mutable_material(), /*on=*/false);

      this->visPub_->Publish(vmsg);
      this->segNames_.push_back(fullName);
    }

    void SetMaterial(msgs::Material *mat, bool on)
    {
      if (!mat) return;
      auto setColor = [](gazebo::msgs::Color *c, double r, double g, double b, double a=1.0)
      {
        c->set_r(r); c->set_g(g); c->set_b(b); c->set_a(a);
      };

      if (on)
      {
        setColor(mat->mutable_ambient(), 0.9, 0.4, 0.0);
        setColor(mat->mutable_diffuse(), 1.0, 0.5, 0.1);
        setColor(mat->mutable_emissive(), 1.0, 0.5, 0.1);
      }
      else
      {
        setColor(mat->mutable_ambient(), 0.05, 0.05, 0.05);
        setColor(mat->mutable_diffuse(), 0.06, 0.06, 0.06);
        setColor(mat->mutable_emissive(), 0.02, 0.02, 0.02);
      }
    }

    // 숫자 → 켜질 세그먼트(7-seg) [a,b,c,d,e,f,g] 순
    void ApplyDigit(int digit)
    {
      std::array<bool,7> segs{false,false,false,false,false,false,false};
      switch (digit)
      {
        case 1: segs = {false, true,  true,  false, false, false, false}; break;
        case 2: segs = {true,  true,  false, true,  true,  false, true }; break;
        case 3: segs = {true,  true,  true,  true,  false, false, true }; break;
        case 4: segs = {false, true,  true,  false, false, true,  true }; break;
        default: segs = {false, true,  true,  false, false, false, false}; break;
      }

      const std::string parent = this->visual_ ? this->visual_->Name() : "";

      for (size_t i = 0; i < this->segNames_.size() && i < segs.size(); ++i)
      {
        msgs::Visual vmsg;
        vmsg.set_name(this->segNames_[i]);
        if (!parent.empty())
          vmsg.set_parent_name(parent);  // 필수: 부모 비주얼 이름 설정
        this->SetMaterial(vmsg.mutable_material(), segs[i]);
        this->visPub_->Publish(vmsg);
      }
    }

  private:
    rendering::VisualPtr visual_;
    transport::NodePtr gzNode_;
    transport::PublisherPtr visPub_;
    event::ConnectionPtr updateConn_;

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
    std::thread rosSpin_;
    std::atomic<bool> stop_{false};

    std::vector<std::string> segNames_;
    std::atomic<int> desiredDigit_{1};
    int currentDigit_{-1};

    // 임계값/오프셋 파라미터 (SDF로 변경 가능)
    double h2_{1.05};
    double h3_{2.05};
    double h4_{3.10};
    double tol_{0.06};       // 허용 오차
    double frontOffsetY_{-0.02}; // 패널 전면이 -Y라면 음수로 설정
  };

  // 등록할 클래스명과 일치시킴
  GZ_REGISTER_VISUAL_PLUGIN(PanelDisplayVisualPlugin)
} // namespace gazebo