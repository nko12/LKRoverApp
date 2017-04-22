#include <ros.h>
#include <ArduinoHardware.h>

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

#include <Servo.h> 
#include <ros.h>
#include <lk_rover/AllPWMs.h>
#include <lk_rover/AllEncoders.h>

ros::NodeHandle  nh;

Servo frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel,
  leftDumpLift, rightDumpLift, leftLadderLift, rightLadderLift,
  ladderSpin;


void servo_cb( const lk_rover::AllPWMs& cmd_msg){
  // TODO
  ladderSpin.write(cmd_msg.front_left);
  digitalWrite(13, HIGH-digitalRead(13)); 
}

lk_rover::AllEncoders encoderVals;

ros::Subscriber<lk_rover::AllPWMs> wheelSub("wheels", servo_cb);
ros::Publisher encoderPub("encoders", &encoderVals);

class QuadatureEncoder {
public:
  QuadatureEncoder(int pinA_, int pinB_, long long unsigned int& ref): pinA(pinA_), pinB(pinB_), count(ref) {}
  
  void init() {
    a = digitalRead(pinA) == HIGH;
    count = 0;
  }
  
  void readEncoders() {
    boolean newA = digitalRead(pinA) == HIGH;
    if (!a && newA) {
      if (digitalRead(pinB) == LOW) {
        --count;
      } else {
        ++count;
      }
    }
    a = newA;
  }
  
  long long unsigned int &count;
private:
  boolean a;
  const int pinA, pinB;
};

const int numQuadEncoders = 4;

QuadatureEncoder encoders[numQuadEncoders] = {
  QuadatureEncoder(22, 23, encoderVals.front_left_enc), // TODO check pins
  QuadatureEncoder(24, 25, encoderVals.front_right_enc),
  QuadatureEncoder(26, 27, encoderVals.back_left_enc),
  QuadatureEncoder(28, 29, encoderVals.back_right_enc),
};

void getEncoderVals() {
  for (int i = 0; i < numQuadEncoders; ++i) {
    QuadatureEncoder& enc = encoders[i];
    enc.readEncoders();
  }
}

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(wheelSub);
  nh.advertise(encoderPub);
  
  for (int i = 0; i < numQuadEncoders; ++i) {
    QuadatureEncoder& enc = encoders[i];
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
}

void loop() {
  getEncoderVals();
  encoderPub.publish(&encoderVals);
  nh.spinOnce();
  delay(1);
}
