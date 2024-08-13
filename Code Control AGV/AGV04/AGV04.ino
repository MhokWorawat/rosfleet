#include <ArduinoHardware.h>
#include <PinChangeInterrupt.h>
#include <AFMotor.h>
#include <PID_v1.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt32MultiArray.h>
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
const int PPR = 1250;
const double wheelRadius = 0.028;      // in meters
const double wheelBaseWidth = 0.11;    // in meters
const double wheelBaseLength = 0.095;  // in meters

const double min_speed_cmd = 0.176;    // in meters/second
const double speed_to_pwm_ratio[] = {0.000774, 0.000710, 0.000903, 0.000452};

#define PI 3.14159
//--------------------------------------[ Define Device Pin ]
#define encoderPin1A 53  // FL
#define encoderPin1B 25  // FL
#define encoderPin2A 51  // FR
#define encoderPin2B 23  // FR
#define encoderPin3A 50  // BL
#define encoderPin3B 22  // BL
#define encoderPin4A 52  // BR
#define encoderPin4B 24  // BR
#define EMERGENCY_SWITCH_PIN 49
#define SUCCESS_WORK_PIN 48
#define BUZZER 46

//--------------------------------------[ Variable : Motor Pin (L, R)]
AF_DCMotor motor1(3);  // M1
AF_DCMotor motor2(2);  // M2
AF_DCMotor motor3(4);  // M3
AF_DCMotor motor4(1);  // M4

//--------------------------------------[ Variable : AGV Speed in m/s ]
double speed_req_motor[] = {0, 0, 0, 0};
double speed_cmd_motor[] = {0, 0, 0, 0};
double speed_cur_motor[] = {0, 0, 0, 0};
int PWM_motor[] = {0, 0, 0, 0};

const double max_speed = 0.25;         // in meters/second

double linear_x;
double linear_y;
double angular_z;

//--------------------------------------[ Variable : Encoder Counter ]
volatile long CounterEncoder1 = 0;
volatile long CounterEncoder2 = 0;
volatile long CounterEncoder3 = 0;
volatile long CounterEncoder4 = 0;

//--------------------------------------[ Variable : Time ]
unsigned long currentTime = 0;
unsigned long lastTime = 0;
unsigned long deltaTime = 0;
const int loopTime = 100;

//--------------------------------------[ Variable : Loop ]
const byte NoCommuLoopsMax = 10;
unsigned int NoCommuLoops = 0;

//--------------------------------------[ Variable : PID]
const double PID_param[] = { 2.5, 0, 0 };
PID PID_Motor1(&speed_cur_motor[0], &speed_cmd_motor[0], &speed_req_motor[0], PID_param[0], PID_param[1], PID_param[2], DIRECT);
PID PID_Motor2(&speed_cur_motor[1], &speed_cmd_motor[1], &speed_req_motor[1], PID_param[0], PID_param[1], PID_param[2], DIRECT);
PID PID_Motor3(&speed_cur_motor[2], &speed_cmd_motor[2], &speed_req_motor[2], PID_param[0], PID_param[1], PID_param[2], DIRECT);
PID PID_Motor4(&speed_cur_motor[3], &speed_cmd_motor[3], &speed_req_motor[3], PID_param[0], PID_param[1], PID_param[2], DIRECT);

//[ Function ] ---------------------------------------------

void AGV_cmd_vel(const geometry_msgs::Twist& cmd_vel) {
  double linearX = cmd_vel.linear.x;
  double linearY = cmd_vel.linear.y;
  double angularZ = cmd_vel.angular.z;

  speed_req_motor[0] = (linearX - linearY - ((wheelBaseWidth + wheelBaseLength) * angularZ));
  speed_req_motor[1] = (linearX + linearY + ((wheelBaseWidth + wheelBaseLength) * angularZ));
  speed_req_motor[2] = (linearX + linearY - ((wheelBaseWidth + wheelBaseLength) * angularZ));
  speed_req_motor[3] = (linearX - linearY + ((wheelBaseWidth + wheelBaseLength) * angularZ));
}
void Buzzer(const std_msgs::Bool& buzzer_msg) {
  digitalWrite(BUZZER, buzzer_msg.data ? HIGH : LOW);
}
ros::Subscriber<geometry_msgs::Twist> AGVsubscriber("cmd_vel", AGV_cmd_vel);
ros::Subscriber<std_msgs::Bool> BuzzerSubscriber("buzzer", Buzzer);

void UpdateEncoder1() {
  CounterEncoder1 += (digitalRead(encoderPin1A) == digitalRead(encoderPin1B)) ? -1 : 1;
}

void UpdateEncoder2() {
  CounterEncoder2 += (digitalRead(encoderPin2A) == digitalRead(encoderPin2B)) ? 1 : -1;
}

void UpdateEncoder3() {
  CounterEncoder3 += (digitalRead(encoderPin3A) == digitalRead(encoderPin3B)) ? -1 : 1;
}

void UpdateEncoder4() {
  CounterEncoder4 += (digitalRead(encoderPin4A) == digitalRead(encoderPin4B)) ? 1 : -1;
}

void UpdateCurrentSpeed() {
  double RPS_Motor1 = (CounterEncoder1 * (1000 / loopTime)) / PPR;
  speed_cur_motor[0] = wheelRadius * 2.0 * PI * RPS_Motor1;

  double RPS_Motor2 = (CounterEncoder2 * (1000 / loopTime)) / PPR;
  speed_cur_motor[1] = wheelRadius * 2.0 * PI * RPS_Motor2;

  double RPS_Motor3 = (CounterEncoder3 * (1000 / loopTime)) / PPR;
  speed_cur_motor[2] = wheelRadius * 2.0 * PI * RPS_Motor3;

  double RPS_Motor4 = (CounterEncoder4 * (1000 / loopTime)) / PPR;
  speed_cur_motor[3] = wheelRadius * 2.0 * PI * RPS_Motor4;
}

void calculateVelocity(){
  linear_x = (speed_cur_motor[0] + speed_cur_motor[1] + speed_cur_motor[2] + speed_cur_motor[3]) / 4;
  linear_y = (-speed_cur_motor[0] + speed_cur_motor[1] + speed_cur_motor[2] - speed_cur_motor[3]) / 4;
  angular_z = (-speed_cur_motor[0] + speed_cur_motor[1] - speed_cur_motor[2] + speed_cur_motor[3]) / (4 * (wheelBaseWidth + wheelBaseLength));
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
  encoder_msg.data_length = 4;
  encoder_msg.data = (uint32_t*) realloc(encoder_msg.data, encoder_msg.data_length * sizeof(uint32_t));
  encoder_msg.data[0] = CounterEncoder1;
  encoder_msg.data[1] = CounterEncoder2;
  encoder_msg.data[2] = CounterEncoder3;
  encoder_msg.data[3] = CounterEncoder4;


  Emergen_pub.publish(&emergen_msg);
  Success_pub.publish(&success_msg);
  Velocity_pub.publish(&velocity_msg);
  Encoder_pub.publish(&encoder_msg);
}

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
  
  PID_Motor1.SetSampleTime(loopTime);
  PID_Motor1.SetOutputLimits(-max_speed, max_speed);
  PID_Motor1.SetMode(AUTOMATIC);

  PID_Motor2.SetSampleTime(loopTime);
  PID_Motor2.SetOutputLimits(-max_speed, max_speed);
  PID_Motor2.SetMode(AUTOMATIC);

  PID_Motor3.SetSampleTime(loopTime);
  PID_Motor3.SetOutputLimits(-max_speed, max_speed);
  PID_Motor3.SetMode(AUTOMATIC);

  PID_Motor4.SetSampleTime(loopTime);
  PID_Motor4.SetOutputLimits(-max_speed, max_speed);
  PID_Motor4.SetMode(AUTOMATIC);

  pinMode(encoderPin1A, INPUT_PULLUP);
  pinMode(encoderPin1B, INPUT_PULLUP);
  pinMode(encoderPin2A, INPUT_PULLUP);
  pinMode(encoderPin2B, INPUT_PULLUP);
  pinMode(encoderPin3A, INPUT_PULLUP);
  pinMode(encoderPin3B, INPUT_PULLUP);
  pinMode(encoderPin4A, INPUT_PULLUP);
  pinMode(encoderPin4B, INPUT_PULLUP);

  pinMode(EMERGENCY_SWITCH_PIN, INPUT_PULLUP);
  pinMode(SUCCESS_WORK_PIN, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);

  attachPCINT(digitalPinToPCINT(encoderPin1A), UpdateEncoder1, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPin2A), UpdateEncoder2, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPin3A), UpdateEncoder3, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPin4A), UpdateEncoder4, CHANGE);

  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void loop() {
  currentTime = millis();
  deltaTime = currentTime - lastTime;

  if (deltaTime >= loopTime) {
    UpdateCurrentSpeed();
    CounterEncoder1 = 0;
    CounterEncoder2 = 0;
    CounterEncoder3 = 0;
    CounterEncoder4 = 0;

    speed_cmd_motor[0] = constrain(speed_cmd_motor[0], -max_speed, max_speed);
    speed_cmd_motor[1] = constrain(speed_cmd_motor[1], -max_speed, max_speed);
    speed_cmd_motor[2] = constrain(speed_cmd_motor[2], -max_speed, max_speed);
    speed_cmd_motor[3] = constrain(speed_cmd_motor[3], -max_speed, max_speed);

    PID_Motor1.Compute();
    PID_Motor2.Compute();
    PID_Motor3.Compute();
    PID_Motor4.Compute();

    PWM_motor[0] = constrain(((speed_req_motor[0] + sgn(speed_req_motor[0]) * min_speed_cmd) / speed_to_pwm_ratio[0]) + (speed_cmd_motor[0] / speed_to_pwm_ratio[0]), -255, 255);
    PWM_motor[1] = constrain(((speed_req_motor[1] + sgn(speed_req_motor[1]) * min_speed_cmd) / speed_to_pwm_ratio[1]) + (speed_cmd_motor[1] / speed_to_pwm_ratio[1]), -255, 255);
    PWM_motor[2] = constrain(((speed_req_motor[2] + sgn(speed_req_motor[2]) * min_speed_cmd) / speed_to_pwm_ratio[2]) + (speed_cmd_motor[2] / speed_to_pwm_ratio[2]), -255, 255);
    PWM_motor[3] = constrain(((speed_req_motor[3] + sgn(speed_req_motor[3]) * min_speed_cmd) / speed_to_pwm_ratio[3]) + (speed_cmd_motor[3] / speed_to_pwm_ratio[3]), -255, 255);

    lastTime = currentTime;
  }

  if (NoCommuLoops >= NoCommuLoopsMax) {
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    motor3.setSpeed(0);
    motor4.setSpeed(0);
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    motor3.run(RELEASE);
    motor4.run(RELEASE);
  } else {

    if (digitalRead(EMERGENCY_SWITCH_PIN) == LOW){
      emergen_msg.data = "on";
      motor1.setSpeed(0);
      motor2.setSpeed(0);
      motor3.setSpeed(0);
      motor4.setSpeed(0);
      motor1.run(RELEASE);
      motor2.run(RELEASE);
      motor3.run(RELEASE);
      motor4.run(RELEASE);

    } else {
      emergen_msg.data = "off";
      success_msg.data = (digitalRead(SUCCESS_WORK_PIN) == LOW) ? "on" : "off";

      motor1.setSpeed(abs(PWM_motor[0]));
      motor2.setSpeed(abs(PWM_motor[1]));
      motor3.setSpeed(abs(PWM_motor[2]));
      motor4.setSpeed(abs(PWM_motor[3]));

      motor1.run(PWM_motor[0] > 0 ? FORWARD : (PWM_motor[0] < 0 ? BACKWARD : RELEASE));
      motor2.run(PWM_motor[1] > 0 ? FORWARD : (PWM_motor[1] < 0 ? BACKWARD : RELEASE));
      motor3.run(PWM_motor[2] > 0 ? FORWARD : (PWM_motor[2] < 0 ? BACKWARD : RELEASE));
      motor4.run(PWM_motor[3] > 0 ? FORWARD : (PWM_motor[3] < 0 ? BACKWARD : RELEASE));

    }

    NoCommuLoops++;
    if (NoCommuLoops == 65535){
      NoCommuLoops = NoCommuLoopsMax;
    }

  }

  calculateVelocity();
  publisher();
  nh.spinOnce();
}
