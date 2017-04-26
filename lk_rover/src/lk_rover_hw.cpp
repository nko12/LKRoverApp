#include <memory>

#include "lk_rover/common.h"

#include "lk_rover/lk_rover_hw.h"

LKRoverHW::LKRoverHW(ros::NodeHandle nh): nh(nh) {
  pubPwm = nh.advertise<lk_rover::AllPWMs>("pwms", 1);
  subEncoders = nh.subscribe("encoders", 1, LKRoverHW::encoderCb, this);
}

void LKRoverHW::setPWMs(const std::array<double, kNumWheels>& pwms,
    double dumpA, double dumpB, double ladderA, double ladderB, double spin, double flap) {
  lk_rover::AllPWMs allPwms;
  allPwms.bucket_left = dumpA;
  allPwms.bucket_right = dumpB;

  allPwms.bucket_spin = spin;
  allPwms.bucket_flap = flap;

  allPwms.ladder_left = ladderA;
  allPwms.ladder_right = ladderB;

  allPwms.front_left = pwms[0];
  allPwms.back_left = pwms[1];
  allPwms.front_right = pwms[2];
  allPwms.back_right = pwms[3];

  pubPwm.publish(allPwms);
}

void LKRoverHW::getCount(std::array<double, kNumWheels>& encoderValsInRadians,
    double &dumpA, double& dumpB, double &ladderA, double &ladderB) {
  encoderLock.lock();
  encoderValsInRadians[0] = kEncoderToRadians*lastEncoderData.front_left_enc;
  encoderValsInRadians[1] = kEncoderToRadians*lastEncoderData.back_left_enc;
  encoderValsInRadians[2] = kEncoderToRadians*lastEncoderData.front_right_enc;
  encoderValsInRadians[3] = kEncoderToRadians*lastEncoderData.back_right_enc;

  dumpA = lastEncoderData.bucket_left_enc;
  dumpB = lastEncoderData.bucket_right_enc;

  ladderA = lastEncoderData.ladder_left_enc;
  ladderB = lastEncoderData.ladder_right_enc;

  encoderLock.unlock();
}

void LKRoverHW::encoderCb(const lk_rover::AllEncoders& encoders) {
  encoderLock.lock();
  {
    lastEncoderData = encoders;
  }
  encoderLock.unlock();
}

void LKRoverHW::waitForSerial() {
  std::shared_ptr<lk_rover::AllEncoders> msg;
  ROS_INFO("waiting for serial...");
  while(!msg) {
    msg = ros::topic::waitForMessage<lk_rover::AllEncoders>("encoders", ros::Duration(30));
    if (!msg) {
      ROS_WARN("attempt to get encoder message failed");
    }
  }
}
