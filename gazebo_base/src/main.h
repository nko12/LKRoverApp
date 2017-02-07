bool SpawnModel(ros::NodeHandle& nh, ros::NodeHandle& nhPrivate);

constexpr char kJointLeftBackName[] = "base_to_left_back_wheel";
constexpr char kJointLeftFrontName[] = "base_to_left_front_wheel";
constexpr char kJointRightBackName[] = "base_to_right_back_wheel";
constexpr char kJointRightFrontName[] = "base_to_right_front_wheel";

constexpr char kLinkLeftBackName[] = "tesbot::left_back_wheel";
constexpr char kLinkLeftFrontName[] = "tesbot::left_front_wheel";
constexpr char kLinkRightBackName[] = "tesbot::right_back_wheel";
constexpr char kLinkRightFrontName[] = "tesbot::right_front_wheel";

constexpr int kNumJoints = 4;

constexpr int kMaxForceNum = 5;

class BaseController;
class PIDController;


class PIDController {
public:
  float desiredVel, curVel;
  float desiredForce;

  int idx;
protected:
  float curForce;
  int nForceApplications;

  BaseController *parent;
public:
  PIDController(BaseController* p, int idx): 
    parent(p), idx(idx), nForceApplications(0),
    desiredVel(0.0f), curVel(0.0f), desiredForce(0.0f), curForce(0.0f) {}

  PIDController(): PIDController(nullptr, 0) {}
  PIDController(const PIDController& other): PIDController(other.parent, other.idx) {
    desiredVel = other.desiredVel;
    curVel = other.curVel;
    desiredForce = other.desiredForce;
    curForce = other.curForce;
    nForceApplications = other.nForceApplications;
  }

  PIDController& operator=(const PIDController& other) {
    parent = other.parent;
    idx = other.idx;

    desiredVel = other.desiredVel;
    curVel = other.curVel;
    desiredForce = other.desiredForce;
    curForce = other.curForce;
    nForceApplications = other.nForceApplications;
  }

public:
  void pidControl();
  void setForce();
  void clearForce();
  bool addForce(float f);
};


class BaseController {
  friend class PIDController;

protected:
  std::array<PIDController, kNumJoints> pidControllers;
  geometry_msgs::Pose curModelPose;

  ros::ServiceClient &moveJoints, &clearJoints;

public:
  BaseController(ros::ServiceClient &mj, ros::ServiceClient &cj): moveJoints(mj), clearJoints(cj) {
    for (int i = 0; i < kNumJoints; i++) {
      pidControllers[i] = PIDController(this, i);
    }
  }

  void ControllerCallback(const geometry_msgs::Twist& t);
  void LinkStatesCallback(const gazebo_msgs::LinkStates& ls);
  void ModelStatesCallback(const gazebo_msgs::ModelStates& ms);

protected:
  void pidControl();
};


