#include <ArduinoHardware.h>
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
const int PPR = 800;
const double wheelRadius = 0.126;         // in meters
const double wheelBaseLength = 0.517;     // in meters

const double min_speed_cmd = 0.001;       // in meters/second
const double speed_to_pwm_ratio = 0.0001;

//--------------------------------------[ Define Device Pin ]
#define ENCODER_A 3
#define ENCODER_B 2
#define ENCODER_C 18
#define ENCODER_D 19
#define LIMIT_SWITCH_PIN_RIGHT 36
#define LIMIT_SWITCH_PIN_LEFT 37
#define EMERGENCY_SWITCH_PIN 41
#define SUCCESS_WORK_PIN 40
#define BUZZER 46

//--------------------------------------[ Variable : Motor Pin ]
const int MotorPINSpeedF = 7;
const int MotorPINSpeedB = 6;
const int MotorPINSteerR = 4;
const int MotorPINSteerL = 5;

//--------------------------------------[ Variable : AGV Speed in m/s ]
double speed_req;
double speed_cmd;
double speed_cur;
int PWM_motor;

const double max_speed = 0.3; 

//--------------------------------------[ Variable : Steering Angle ]
double steering_req;
double steering_cur;
int PWMsteering = 60;

const int max_steering = 70; 

//--------------------------------------[ Variable : Encoder ]
volatile long CounterEncoderSpeed = 0;
volatile long CounterEncoderSteer = 0;

//--------------------------------------[ Variable : Time ]
unsigned long currentTime = 0;
unsigned long lastTime = 0;
unsigned long deltaTime = 0;
const int loopTime = 100;

//--------------------------------------[ Variable : Loop ]
const byte NoCommuLoopsMax = 10;
unsigned int NoCommuLoops = 0;

//--------------------------------------[ Variable : Mode & State]
enum SetState { setDirection, onLoop };
static SetState state = setDirection;

//--------------------------------------[ Variable : PID]
const double PID_speed_param[] = { 1.3, 0, 0 };
const double PID_steer_param[] = { 1.3, 0, 0 };
PID PID_Speed(&speed_cur, &speed_cmd, &speed_req, PID_speed_param[0], PID_speed_param[1], PID_speed_param[2], DIRECT);

//[ Function ] ---------------------------------------------

void AGV_cmd_vel(const geometry_msgs::Twist& cmd_vel) {
  double linearX = cmd_vel.linear.x;
  double angularZ = cmd_vel.angular.z;

  if (linearX == 0){
    speed_req = 0;
    steering_req = 0;
  } else {
    speed_req = linearX;
    steering_req = constrain(degrees(angularZ), -max_steering, max_steering);
  }

}
void Buzzer(const std_msgs::Bool& buzzer_msg) {
  digitalWrite(BUZZER, buzzer_msg.data ? HIGH : LOW);
}
ros::Subscriber<geometry_msgs::Twist> AGVsubscriber("cmd_vel", AGV_cmd_vel);
ros::Subscriber<std_msgs::Bool> BuzzerSubscriber("buzzer", Buzzer);

void UpdateEncoderSpeed(){
  CounterEncoderSpeed += (digitalRead(ENCODER_C) == digitalRead(ENCODER_D)) ? 1 : -1;
}

void UpdateEncoderSteer(){
  CounterEncoderSteer += (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) ? -1 : 1;
}

void UpdateCurrentSpeed(){
  double RPS = (CounterEncoderSpeed * (1000 / loopTime)) / PPR;
  speed_cur = wheelRadius * 2.0 * PI * RPS;
}

void UpdateCurrentSteer() {
  CounterEncoderSteer = constrain(CounterEncoderSteer, 0, 385);
  steering_cur = map(CounterEncoderSteer, 0, 385, -90, 90);
}

void publisher() {
  // Prepare velocity data
  velocity_msg.linear.x = speed_cur;
  velocity_msg.linear.y = 0.0;
  velocity_msg.linear.z = 0.0;
  velocity_msg.angular.x = 0.0;
  velocity_msg.angular.y = 0.0;
  velocity_msg.angular.z = steering_cur * (PI / 180);

  // Prepare encoder data
  encoder_msg.data_length = 2;
  encoder_msg.data = (uint32_t*) realloc(encoder_msg.data, encoder_msg.data_length * sizeof(uint32_t));
  encoder_msg.data[0] = CounterEncoderSpeed;
  encoder_msg.data[1] = CounterEncoderSteer;

  Emergen_pub.publish(&emergen_msg);
  Success_pub.publish(&success_msg);
  Velocity_pub.publish(&velocity_msg);
  Encoder_pub.publish(&encoder_msg);
}

class motor{

  public:
    int pinF;
    int pinB;

  void setup(){
    pinMode(pinF, OUTPUT);
    pinMode(pinB, OUTPUT);
  }

  void speed(const int PWM, const String InputDirection){
    if(InputDirection == "FORWARD"){
      analogWrite(pinF, abs(PWM));
      analogWrite(pinB, 0);
    } else if (InputDirection == "BACKWARD"){
      analogWrite(pinF, 0);
      analogWrite(pinB, abs(PWM));
    } else if (InputDirection == "STOP"){
      analogWrite(pinF, 0);
      analogWrite(pinB, 0);
    }
  }

};

motor motor;

class steering{

  public:
    int pinR;
    int pinL;

  void setup(){
    pinMode(pinR, OUTPUT);
    pinMode(pinL, OUTPUT);
  }

  void setDirection(const int steering_req_ang){
    int steering_error = steering_req_ang - steering_cur;
    if(steering_error < 0){
      analogWrite(pinR, PWMsteering);
      analogWrite(pinL, 0);
    } else if (steering_error > 0){
      analogWrite(pinR, 0);
      analogWrite(pinL, PWMsteering);
    } else {
      analogWrite(pinR, 0);
      analogWrite(pinL, 0);
    }
  }

};

steering steering;

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

  PID_Speed.SetSampleTime(loopTime);
  PID_Speed.SetOutputLimits(-max_speed, max_speed);
  PID_Speed.SetMode(AUTOMATIC);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_C, INPUT_PULLUP);
  pinMode(ENCODER_D, INPUT_PULLUP);

  pinMode(LIMIT_SWITCH_PIN_LEFT, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_PIN_RIGHT, INPUT_PULLUP);
  pinMode(EMERGENCY_SWITCH_PIN, INPUT_PULLUP); 
  pinMode(SUCCESS_WORK_PIN, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), UpdateEncoderSteer, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_C), UpdateEncoderSpeed, CHANGE);

  motor.pinF = MotorPINSpeedF;
  motor.pinB = MotorPINSpeedB;
  motor.setup();
  motor.speed(0, "STOP");
  steering.pinR = MotorPINSteerR;
  steering.pinL = MotorPINSteerL;
  steering.setup();
  steering.setDirection(0);

}

void loop() {
  currentTime = millis();
  deltaTime = currentTime - lastTime;

  if (deltaTime >= loopTime) {
    UpdateCurrentSpeed();
    CounterEncoderSpeed = 0;
    speed_cmd = constrain(speed_cmd, -max_speed, max_speed);
    PID_Speed.Compute();
    PWM_motor = constrain(((speed_req + sgn(speed_req) * min_speed_cmd) / speed_to_pwm_ratio) + (speed_cmd / speed_to_pwm_ratio), -255, 255);
    lastTime = currentTime;
  }

  switch (state) {
 
    case setDirection:

      if (digitalRead(LIMIT_SWITCH_PIN_LEFT) == HIGH){
        motor.speed(0, "STOP");
        CounterEncoderSteer = 0;
        steering_req = 0;
        state = onLoop;
      } else {
        analogWrite(MotorPINSteerR, 0);
        analogWrite(MotorPINSteerL, PWMsteering);
      }
      break;
    
    case onLoop:
      if (NoCommuLoops >= NoCommuLoopsMax) { 
        motor.speed(0, "STOP");
        steering.setDirection(0);

      } else {
        if (digitalRead(EMERGENCY_SWITCH_PIN) == LOW){
          emergen_msg.data = "on";
          motor.speed(0, "STOP");
          steering.setDirection(0);

        } else {
          emergen_msg.data = "off";
          success_msg.data = (digitalRead(SUCCESS_WORK_PIN) == LOW) ? "on" : "off";

          steering.setDirection(steering_req);
          motor.speed(PWM_motor, PWM_motor > 0 ? "FORWARD" : (PWM_motor < 0 ? "BACKWARD" : "STOP"));
        }
      }
      break;
        
  }

  publisher();
  nh.spinOnce();

}
