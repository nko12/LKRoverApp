#define USE_USBCON
#define USB_CON
/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <ArduinoHardware.h>

#include <Servo.h> 

#include <lk_rover/AllPWMs.h>
#include <lk_rover/AllEncoders.h>

ros::NodeHandle  nh;

Servo frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel,
  leftDumpLift, rightDumpLift, leftLadderLift, rightLadderLift,
  ladderSpin, bucketFlap;


void servo_cb( const lk_rover::AllPWMs& cmd_msg){
  // TODO
  frontLeftWheel.write(cmd_msg.front_left);
  frontRightWheel.write(cmd_msg.front_right);
  backLeftWheel.write(cmd_msg.back_left);
  backRightWheel.write(cmd_msg.back_right);
  leftDumpLift.write(cmd_msg.bucket_left);
  rightDumpLift.write(cmd_msg.bucket_right);
  leftLadderLift.write(cmd_msg.ladder_left);
  rightLadderLift.write(cmd_msg.ladder_right);
  ladderSpin.write(cmd_msg.bucket_spin);
  bucketFlap.write(cms_msg.bucket_flap);

  // digitalWrite(13, HIGH-digitalRead(13)); 
}

lk_rover::AllEncoders encoderVals;

ros::Subscriber<lk_rover::AllPWMs> wheelSub("wheels", servo_cb);
ros::Publisher encoderPub("encoders", &encoderVals);

class Encoder {
public:
  virtual void init();
  virtual void readEncoders();
};

template <int A, int B> class QuadatureEncoder: public Encoder {
public:
  QuadatureEncoder(long long unsigned int& ref): count(ref) {}

  static void encoderISR() {
    if (digitalRead(B) == HIGH) {
      ++isrCount;
    } else {
      --isrCount;
    }
  }
  
  void init() {
    pinMode(A, INPUT);
    pinMode(B, INPUT);
    count = 0;
    oldIsrCount = 0;
    attachInterrupt(digitalPinToInterrupt(A), encoderISR, RISING);
  }
  
  void readEncoders() {
    const int curIsrCount = isrCount;
    const int delta = curIsrCount - oldIsrCount;
    count += static_cast<long long int>(delta);
    oldIsrCount = curIsrCount;
  }

  long long unsigned int &count;
private:
  static volatile int isrCount;
  int oldIsrCount;
};
template <int A, int B> volatile int QuadatureEncoder<A, B>::isrCount = 0;

const int numQuadEncoders = 4;

QuadatureEncoder<22, 23> frontLeftEnc(encoderVals.front_left_enc);
QuadatureEncoder<24, 25> frontRightEnc(encoderVals.front_right_enc);
QuadatureEncoder<26, 27> backLeftEnc(encoderVals.back_left_enc);
QuadatureEncoder<28, 29> backRightEnc(encoderVals.back_right_enc);

Encoder *encoders[numQuadEncoders] = {
  &frontLeftEnc, &frontRightEnc, &backLeftEnc, &backRightEnc
};

void getEncoderVals() {
  for (int i = 0; i < numQuadEncoders; ++i) {
    Encoder& enc = *encoders[i];
    enc.readEncoders();
  }
}

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(wheelSub);
  nh.advertise(encoderPub);
  
  for (int i = 0; i < numQuadEncoders; ++i) {
    Encoder& enc = *encoders[i];
    enc.init();
  }
  frontLeftWheel.attach(2);
  frontRightWheel.attach(3);
  backLeftWheel.attach(4);
  backRightWheel.attach(5);
  leftDumpLift.attach(6);
  rightDumpLift.attach(7);
  leftLadderLift.attach(8);
  rightLadderLift.attach(9);
  ladderSpin.attach(10);
  bucketFlap.attach(11);
}

void loop() {
  getEncoderVals();
  encoderPub.publish(&encoderVals);
  nh.spinOnce();
  delay(1);
}
