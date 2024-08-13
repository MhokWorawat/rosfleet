#include <ArduinoHardware.h>
#include <PinChangeInterrupt.h>
#include <PID_v1.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

//--------------------------------------[ ROS ]
ros::NodeHandle nh;
geometry_msgs::Twist velocity_msg;
std_msgs::UInt32MultiArray encoder_msg;
ros::Publisher Velocity_pub("Velocity", &velocity_msg);
ros::Publisher Encoder_pub("Encoder", &encoder_msg);

std_msgs::String emergen_msg;
std_msgs::String success_msg;
ros::Publisher Emergen_pub("Emergen", &emergen_msg);
ros::Publisher Success_pub("Success", &success_msg);

//--------------------------------------[ AGV04 Specific Constants ]
const int PPR = 95;
const double wheelRadius = 0.095;      // in meters
const double wheelBaseWidth = 0.315;    // in meters

const double min_speed_cmd = 0.001;    // in meters/second
const double speed_to_pwm_ratio[] = {0.0001, 0.0002};

#define PI 3.14159
//--------------------------------------[ Define Device Pin ]
#define ENCODERPINL 2
#define ENCODERPINR 3
#define EMERGENCY_SWITCH_PIN 41
#define SUCCESS_WORK_PIN 40
#define BUZZER 46

//--------------------------------------[ Variable : Motor Pin (L, R)]
const int MotorPin[] = {4, 5};
const int DirectionPin[] = {10, 11};
const int Break[] = {8, 9};

//--------------------------------------[ Variable : AGV Speed in m/s ]
double speed_req_motorL;
double speed_req_motorR;
double speed_cmd_motorL;
double speed_cmd_motorR;
double speed_cur_motorL;
double speed_cur_motorR;
int PWM_motor[] = {0, 0};

const double max_speed = 0.3;         // in meters/second

double linear_x;
double linear_y;
double angular_z;

//--------------------------------------[ Variable : Encoder Counter ]
volatile long CounterEncoderL = 0;
volatile long CounterEncoderR = 0;

enum Direction { FORWARD, BACKWARD, STOP };
Direction DirectionEncoderL = STOP;
Direction DirectionEncoderR = STOP;

//--------------------------------------[ Variable : Time ]
unsigned long currentTime = 0;
unsigned long lastTime = 0;
unsigned long deltaTime = 0;
const int loopTime = 100;

//--------------------------------------[ Variable : Loop ]
const byte NoCommuLoopsMax = 10;
unsigned int NoCommuLoops = 0;

//--------------------------------------[ Variable : PID]
const double PID_param[] = { 2.0, 0.0, 0.0 };
PID PID_MotorL(&speed_cur_motorL, &speed_cmd_motorL, &speed_req_motorL, PID_param[0], PID_param[1], PID_param[2], DIRECT);
PID PID_MotorR(&speed_cur_motorR, &speed_cmd_motorR, &speed_req_motorR, PID_param[0], PID_param[1], PID_param[2], DIRECT);

//[ Function ] ---------------------------------------------

void AGV_cmd_vel(const geometry_msgs::Twist& cmd_vel) {
  double linearX = cmd_vel.linear.x;
  double angularZ = cmd_vel.angular.z;

  speed_req_motorL = linearX - ((angularZ * wheelBaseWidth) / 2);
  speed_req_motorR = linearX + ((angularZ * wheelBaseWidth) / 2);
}

void Buzzer(const std_msgs::Bool& buzzer_msg) {
  digitalWrite(BUZZER, buzzer_msg.data ? HIGH : LOW);
}

ros::Subscriber<geometry_msgs::Twist> AGVsubscriber("cmd_vel", AGV_cmd_vel);
ros::Subscriber<std_msgs::Bool> BuzzerSubscriber("buzzer", Buzzer);

void UpdateEncoderSpeedL() {
  CounterEncoderL += (DirectionEncoderL == FORWARD) - (DirectionEncoderL == BACKWARD);
}

void UpdateEncoderSpeedR() {
  CounterEncoderR += (DirectionEncoderR == FORWARD) - (DirectionEncoderR == BACKWARD);
}

void UpdateCurrentSpeed() {
  double RPS_MotorL = (CounterEncoderL * (1000 / loopTime)) / PPR;
  speed_cur_motorL = wheelRadius * 2.0 * PI * RPS_MotorL;

  double RPS_MotorR = (CounterEncoderR * (1000 / loopTime)) / PPR;
  speed_cur_motorR = wheelRadius * 2.0 * PI * RPS_MotorR;
}

void calculateVelocity(){
  linear_x = (speed_cur_motorR + speed_cur_motorL) / 2;
  linear_y = 0;
  angular_z = (speed_cur_motorR - speed_cur_motorL) / wheelBaseWidth;
}

void publisher() {
  // Prepare velocity data
  velocity_msg.linear.x = linear_x;
  velocity_msg.linear.y = linear_y;
  velocity_msg.linear.z = 0.0;
  velocity_msg.angular.x = 0.0;
  velocity_msg.angular.y = 0.0;
  velocity_msg.angular.z = angular_z;

  // Prepare encoder data
  encoder_msg.data_length = 2;
  encoder_msg.data = (uint32_t*) realloc(encoder_msg.data, encoder_msg.data_length * sizeof(uint32_t));
  encoder_msg.data[0] = CounterEncoderL;
  encoder_msg.data[1] = CounterEncoderR;

  Emergen_pub.publish(&emergen_msg);
  Success_pub.publish(&success_msg);
  Velocity_pub.publish(&velocity_msg);
  Encoder_pub.publish(&encoder_msg);
}

class Motor {
  public:
    int Pin;
    int DirectionPin;
    enum Direction { FORWARD, BACKWARD, STOP };
    Direction DirectionEncoder = STOP;
  
  void setup() {
    pinMode(Pin, OUTPUT);
    pinMode(DirectionPin, OUTPUT);
  }

  void speed(const int PWM, Direction inputDirection) {
    DirectionEncoder = inputDirection;
    switch (inputDirection) {
      case FORWARD:
        digitalWrite(DirectionPin, HIGH);
        break;
      case BACKWARD:
        digitalWrite(DirectionPin, LOW);
        break;
      case STOP:
        analogWrite(Pin, 0);
        return;
    }
    analogWrite(Pin, abs(PWM));
  }
};

// Declare instances of the Motor class
Motor motorL, motorR;

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

void setup() {
  Serial.begin(115200);
  
  nh.initNode();
  nh.advertise(Velocity_pub);
  nh.advertise(Encoder_pub);
  nh.advertise(Emergen_pub);
  nh.advertise(Success_pub);

  PID_MotorL.SetSampleTime(loopTime);
  PID_MotorL.SetOutputLimits(-max_speed, max_speed);
  PID_MotorL.SetMode(AUTOMATIC);

  PID_MotorR.SetSampleTime(loopTime);
  PID_MotorR.SetOutputLimits(-max_speed, max_speed);
  PID_MotorR.SetMode(AUTOMATIC);

  pinMode(ENCODERPINL, INPUT_PULLUP);
  pinMode(ENCODERPINR, INPUT_PULLUP);

  pinMode(EMERGENCY_SWITCH_PIN, INPUT_PULLUP); 
  pinMode(SUCCESS_WORK_PIN, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODERPINL), UpdateEncoderSpeedL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODERPINR), UpdateEncoderSpeedR, CHANGE);

  motorL.Pin = MotorPin[0];
  motorR.Pin = MotorPin[1];
  motorL.DirectionPin = DirectionPin[0];
  motorR.DirectionPin = DirectionPin[1];
  motorL.setup();
  motorR.setup();
  motorL.speed(0, Motor::STOP);
  motorR.speed(0, Motor::STOP);
}

void loop() {
  currentTime = millis();
  deltaTime = currentTime - lastTime;

  if (deltaTime >= loopTime) {
    UpdateCurrentSpeed();
    CounterEncoderL = 0;
    CounterEncoderR = 0;

    speed_cmd_motorL = constrain(speed_cmd_motorL, -max_speed, max_speed);
    speed_cmd_motorR = constrain(speed_cmd_motorR, -max_speed, max_speed);

    PID_MotorL.Compute();
    PID_MotorR.Compute();

    PWM_motor[0] = constrain(((speed_req_motorL + sgn(speed_req_motorL) * min_speed_cmd) / speed_to_pwm_ratio[0]) + (speed_cmd_motorL / speed_to_pwm_ratio[0]), -255, 255);
    PWM_motor[1] = constrain(((speed_req_motorR + sgn(speed_req_motorR) * min_speed_cmd) / speed_to_pwm_ratio[1]) + (speed_cmd_motorR / speed_to_pwm_ratio[1]), -255, 255);

    lastTime = currentTime;
  }

  if (NoCommuLoops >= NoCommuLoopsMax) { 
    motorL.speed(0, Motor::STOP);
    motorR.speed(0, Motor::STOP);
  } else {
    
    if (digitalRead(EMERGENCY_SWITCH_PIN) == LOW) {
      emergen_msg.data = "on";
      motorL.speed(0, Motor::STOP);
      motorR.speed(0, Motor::STOP);
    } else {
      emergen_msg.data = "off";
      success_msg.data = (digitalRead(SUCCESS_WORK_PIN) == LOW) ? "on" : "off";

      motorL.speed(PWM_motor[0], PWM_motor[0] > 0 ? Motor::FORWARD : (PWM_motor[0] < 0 ? Motor::BACKWARD : Motor::STOP));
      motorR.speed(PWM_motor[1], PWM_motor[1] > 0 ? Motor::FORWARD : (PWM_motor[1] < 0 ? Motor::BACKWARD : Motor::STOP));
    }

    NoCommuLoops++;
    if (NoCommuLoops == 65535) {
      NoCommuLoops = NoCommuLoopsMax;
    }
  }

  calculateVelocity();
  publisher();
  nh.spinOnce();
}
