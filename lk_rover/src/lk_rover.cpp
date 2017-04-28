#include "lk_rover/lk_rover.h"

TwinJoints::TwinJoints(const ActuatorConfigs &ac):
  a(ac.left), b(ac.right), diffGain(ac.diffGain), length(ac.length), lastA(0.0), lastB(0.0) {}

double TwinJoints::getProcessedEncoder(double inA, double inB) {
  // store the values so that they can be used for computing the necessary
  // PWMs
  lastA = (inA - a.min)/(a.max - a.min);
  lastB = (inB - b.min)/(b.max - b.min);

  // ROS_INFO("a b %lf %lf", lastA, lastB);
  // use the average as the position of the virtual actuator
  // returns the virtual distance in meters
  return length*(lastA + lastB)/2;
}

void TwinJoints::getProcessedPwms(double pwm, double &outA, double &outB) {
  // basically adjust the pwm output based on the discrepancy between the two
  // measured distances
  // ROS_INFO("pwm %lf", pwm);
  double diff = lastA - lastB;
  if (std::abs(diff) <= 0.001) diff = 0.0;
  outA = a.gain * (pwm - diffGain*diff);
  outB = b.gain * (pwm + diffGain*diff);
  ROS_INFO("diff %lf %lf %lf %lf", diffGain, diff, outA, outB);
}

double dummyVal = 0.0;

LKRover::LKRover(std::shared_ptr<LKHW> hw_, ActuatorConfigs& dump, ActuatorConfigs& ladder):
    virtualDump(dump),
    virtualLadder(ladder),
    wheelAccels(),
    wheelPoss(),
    wheelVels(),
    wheelPwms(),
    dumpPos(0.0),
    dumpVel(0.0),
    dumpAccel(0.0),
    dumpPwm(0.0),
    ladderPos(0.0),
    ladderVel(0.0),
    ladderAccel(0.0),
    ladderPwm(0.0),
    hw(hw_),
    lastTime(ros::Time::now()),
    spin(0.0),
    flap(0.0) {
  
  for (int i = 0; i < kNumWheels; ++i) {
    auto jsh = hardware_interface::JointStateHandle(
        kWheelNames[i], &wheelPoss[i], &wheelVels[i], &wheelAccels[i]);
    auto jh = hardware_interface::JointHandle(jsh, &wheelPwms[i]);

    jsi.registerHandle(jsh);
    vji.registerHandle(jh);
  }

  auto dsh = hardware_interface::JointStateHandle(
      "dump", &dumpPos, &dumpVel, &dumpAccel);
  auto dh = hardware_interface::JointHandle(dsh, &dumpPwm);
  jsi.registerHandle(dsh);
  vji.registerHandle(dh);

  auto lsh = hardware_interface::JointStateHandle(
      "ladder", &ladderPos, &ladderVel, &ladderAccel);
  auto lh = hardware_interface::JointHandle(lsh, &ladderPwm);
  jsi.registerHandle(lsh);
  vji.registerHandle(lh);

  auto ssh = hardware_interface::JointStateHandle(
      "spin", &dummyVal, &dummyVal, &dummyVal);
  auto sjh = hardware_interface::JointHandle(ssh, &spin);
  eji.registerHandle(sjh);

  auto fsh = hardware_interface::JointStateHandle(
      "flap", &dummyVal, &dummyVal, &dummyVal);
  auto fjh = hardware_interface::JointHandle(fsh, &flap);
  pji.registerHandle(fjh);

  registerInterface(&vji);
  registerInterface(&jsi);
  registerInterface(&pji);
  registerInterface(&eji);

  // initialize all the PWM values
  double dumpA, dumpB, ladderA, ladderB;
  hw->getCount(wheelPoss, dumpA, dumpB, ladderA, ladderB);
  dumpPos = virtualDump.getProcessedEncoder(dumpA, dumpB);
  ladderPos = virtualDump.getProcessedEncoder(ladderA, ladderB);
}

LKRover::~LKRover() {
}

void LKRover::write() {
  double dumpA, dumpB, ladderA, ladderB;
  virtualDump.getProcessedPwms(dumpPwm, dumpA, dumpB);
  virtualLadder.getProcessedPwms(ladderPwm, ladderA, ladderB);
  // ROS_INFO("pwms %lf %lf %lf %lf %lf %lf",  dumpA, dumpB, ladderA, ladderB, dumpPwm, ladderPwm);

  hw->setPWMs(wheelPwms, dumpA, dumpB, ladderA, ladderB, spin, flap);
}

void LKRover::read() {
  std::array<double, kNumWheels> oldPos = wheelPoss;
  std::array<double, kNumWheels> oldVels = wheelVels;
  double oldDumpPos = dumpPos, oldLadderPos = ladderPos;
  double oldDumpVel = dumpVel, oldLadderVel = ladderVel;

  double dumpA, dumpB, ladderA, ladderB;

  hw->getCount(wheelPoss, dumpA, dumpB, ladderA, ladderB);
  dumpPos = virtualDump.getProcessedEncoder(dumpA, dumpB);
  ladderPos = virtualLadder.getProcessedEncoder(ladderA, ladderB);

  ros::Time curTime = ros::Time::now();
  auto d = [&](double newPos, double old) -> double {
    return (newPos - old)/(curTime - lastTime).toSec();
  };
  for (int i = 0; i < kNumWheels; i++) {
    wheelVels[i] = d(wheelPoss[i], oldPos[i]);
  }
  for (int i = 0; i < kNumWheels; i++) {
    wheelAccels[i] = d(wheelVels[i], oldVels[i]);
  }

  dumpVel = d(dumpPos, oldDumpPos);
  ladderVel = d(ladderPos, oldLadderPos);
  dumpAccel = d(dumpVel, oldDumpVel);
  ladderAccel = d(ladderVel, oldLadderVel);

  lastTime = curTime;
}
