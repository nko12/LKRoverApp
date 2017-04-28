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

volatile bool received = false;
Servo frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel,
  leftDumpLift, rightDumpLift, leftLadderLift, rightLadderLift,
  ladderSpin, bucketFlap;

int clamped(float in) {
  int out = (in * 500) + 1500;
  if (out > 2000) out = 2000;
  if (out < 1000) out = 1000;
  return out;
}

void servo_cb( const lk_rover::AllPWMs& cmd_msg){
  // TODO
  frontLeftWheel.writeMicroseconds(clamped(cmd_msg.front_left));
  frontRightWheel.writeMicroseconds(clamped(cmd_msg.front_right));
  backLeftWheel.writeMicroseconds(clamped(cmd_msg.back_left));
  backRightWheel.writeMicroseconds(clamped(cmd_msg.back_right));
  leftDumpLift.writeMicroseconds(clamped(cmd_msg.bucket_left));
  rightDumpLift.writeMicroseconds(clamped(cmd_msg.bucket_right));
  leftLadderLift.writeMicroseconds(clamped(cmd_msg.ladder_left));
  rightLadderLift.writeMicroseconds(clamped(cmd_msg.ladder_right));
  ladderSpin.writeMicroseconds(clamped(cmd_msg.bucket_spin));
  bucketFlap.writeMicroseconds(clamped(cmd_msg.bucket_flap));

  received = true;
  // digitalWrite(13, HIGH-digitalRead(13)); 
}

lk_rover::AllEncoders encoderVals;

ros::Subscriber<lk_rover::AllPWMs> wheelSub("pwms", servo_cb);
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

class PotSensor: public Encoder {
public:
  PotSensor(int pin, float &output): pin(pin), ref(output) {;}
  void init() {
    pinMode(pin, INPUT); 
  }
  void readEncoders() {
    ref = static_cast<float>(analogRead(pin));
  }

  const int pin;
  float &ref;
};

const int numQuadEncoders = 4;
const int numPotSensors = 4;

QuadatureEncoder<22, 23> frontLeftEnc(encoderVals.front_left_enc);
QuadatureEncoder<24, 25> frontRightEnc(encoderVals.front_right_enc);
QuadatureEncoder<26, 27> backLeftEnc(encoderVals.back_left_enc);
QuadatureEncoder<28, 29> backRightEnc(encoderVals.back_right_enc);

PotSensor bucketLeftEnc(A0, encoderVals.bucket_left_enc);
PotSensor bucketRightEnc(A1, encoderVals.bucket_right_enc);
PotSensor ladderLeftEnc(A2, encoderVals.ladder_left_enc);
PotSensor ladderRightEnc(A3, encoderVals.ladder_right_enc);

Encoder *encoders[numQuadEncoders + numPotSensors] = {
  &frontLeftEnc, &frontRightEnc, &backLeftEnc, &backRightEnc,
  &bucketLeftEnc, &bucketRightEnc, &ladderLeftEnc, &ladderRightEnc
};

void getEncoderVals() {
  for (int i = 0; i < sizeof(encoders)/sizeof(encoders[0]); ++i) {
    Encoder& enc = *encoders[i];
    enc.readEncoders();
  }
}

void setup(){
  analogReadResolution(16);
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

void killMotors() {
  frontLeftWheel.writeMicroseconds(1500);
  frontRightWheel.writeMicroseconds(1500);
  backLeftWheel.writeMicroseconds(1500);
  backRightWheel.writeMicroseconds(1500);
  leftDumpLift.writeMicroseconds(1500);
  rightDumpLift.writeMicroseconds(1500);
  leftLadderLift.writeMicroseconds(1500);
  rightLadderLift.writeMicroseconds(1500);
  ladderSpin.writeMicroseconds(1500);
}

int timeout = 0;
void loop() {
  getEncoderVals();
  encoderPub.publish(&encoderVals);
  nh.spinOnce();
  if (!received) {
    ++timeout;
    if (timeout > 500) {
      killMotors();
    }
  } else {
    timeout = 0;
    received = false;
  }
  delay(1);
}
