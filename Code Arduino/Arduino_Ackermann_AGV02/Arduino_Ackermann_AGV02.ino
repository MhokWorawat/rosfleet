#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

class PIDspeed{
  private:
    float Kp, Ki, Kd;
    float error, error_integral, error_derivative, error_previous;

  public:
    PIDspeed() : Kp(), Ki(), Kd(), error(), error_integral(), error_derivative(), error_previous() {}

    void SetParams(float KpIn, float KiIn, float KdIn) {
      Kp = KpIn; Ki = KiIn; Kd = KdIn;
    }
    
    int Evaluate(float value, float target, float deltaTime) {
      
      error = target - value;
      error_integral += error * deltaTime;
      error_derivative = (error - error_previous) / deltaTime;
      float PWMpid = Kp * error + Ki * error_integral + Kd * error_derivative;

      int PWM = static_cast<int>(fabs(PWMpid));
      PWM = constrain(PWM, 0, 255);

      error_previous = error;
      return PWM;
    }
};

//--------------------------------------[ ROS ]
  ros::NodeHandle nh;

  geometry_msgs::Twist msg;

  std_msgs::Float32 LinearX_msg;
  std_msgs::Float32 LinearY_msg;
  std_msgs::Float32 AngularZ_msg;
  ros::Publisher VelocityX("LinearX", &LinearX_msg);
  ros::Publisher VelocityY("LinearY", &LinearY_msg);
  ros::Publisher VelocityZ("AngularZ", &AngularZ_msg);

  std_msgs::Float32 Speed_msg;
  std_msgs::Float32 Angle_msg;
  ros::Publisher CheckSpeed("Speed", &Speed_msg);
  ros::Publisher CheckAngle("Angle", &Angle_msg);

  std_msgs::Float32 EncSpeed_msg;
  std_msgs::Float32 EncAngle_msg;
  ros::Publisher EncSpeed("EncSpeed", &EncSpeed_msg);
  ros::Publisher EncAngle("EncAngle", &EncAngle_msg);

  std_msgs::String Emergen_msg;
  std_msgs::String Success_msg;
  ros::Publisher Emergen("Emergen", &Emergen_msg);
  ros::Publisher Success("Success", &Success_msg);

//--------------------------------------[ AGV01 Constant ]
#define PI 3.14159
const float wheelRadius = 0.126;    // .M
const float wheelBase = 0.517;  // .M
const int PPR = 800;

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
const int Motor_F = 7;
const int Motor_B = 6;
const int Motor_R = 4;
const int Motor_L = 5;

//--------------------------------------[ Variable : PID control]
const float Kp = 1.2;
const float Ki = 0.0;
const float Kd = 0.0;

PIDspeed PIDspeed;

//--------------------------------------[ Variable : Encoder ]
volatile long CounterEncoderAngle = 0;
volatile long CounterEncoderSpeed = 0;

//--------------------------------------[ Variable : Speed ]
double linearVelocityX;
double linearVelocityY;
double angularVelocityZ;
double TargetSpeed;
double TargetCount;
double speed;
int PWMmotor;

//--------------------------------------[ Variable : Steering Angle ]
double turningRadius;
double TargetSteeringAngle_rad;
double TargetSteeringAngle_deg;
double CurrentSteeringAngle;
int SteeringAngleError;
int PWMsteering = 60;

//--------------------------------------[ Variable : Time ]
unsigned long lastTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime;

//--------------------------------------[ Variable : Mode & State]

enum SetState { setDirection, onLoop };
static SetState state = setDirection;

//[ Function ] ----------------------------------------------
void AGVCallBack(const geometry_msgs::Twist& cmd_vel) {
  linearVelocityX = cmd_vel.linear.x;
  linearVelocityY = cmd_vel.linear.y;
  angularVelocityZ = cmd_vel.angular.z;
}

void BuzzerCallBack(const std_msgs::Bool& buzzer_msg) {
  if (buzzer_msg.data) {
    digitalWrite(BUZZER, HIGH);
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

ros::Subscriber<geometry_msgs::Twist> AGVsubscriber("cmd_vel", AGVCallBack);
ros::Subscriber<std_msgs::Bool> BuzzerSubscriber("buzzer", BuzzerCallBack);

void UpdateEncoderSpeed(){
  CounterEncoderSpeed += (digitalRead(ENCODER_C) == digitalRead(ENCODER_D) ? 1 : -1;
}

void UpdateEncoderAngle(){
  CounterEncoderAngle += (digitalRead(ENCODER_A) == digitalRead(ENCODER_B) ? -1 : 1;
}

void UpdateCurrentSpeed(){
  double RPS = static_cast<double>(CounterEncoderSpeed * 10) / PPR;
  speed = wheelRadius * 2.0 * PI * RPS;
}

void UpdateCurrentAngle(){
  CounterEncoderAngle = constrain(CounterEncoderAngle, 0, 385);
  CurrentSteeringAngle = map(CounterEncoderAngle, 0, 385, 90, -90);
}

//***************************************************************************************************************************************
void setup() {
  PIDspeed.SetParams(Kp, Ki, Kd);

  pinMode(Motor_F, OUTPUT);
  pinMode(Motor_B, OUTPUT);
  pinMode(Motor_R, OUTPUT);
  pinMode(Motor_L, OUTPUT);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_C, INPUT_PULLUP);
  pinMode(ENCODER_D, INPUT_PULLUP);

  pinMode(LIMIT_SWITCH_PIN_LEFT, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_PIN_RIGHT, INPUT_PULLUP);
  pinMode(EMERGENCY_SWITCH_PIN, INPUT_PULLUP); 
  pinMode(SUCCESS_WORK_PIN, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), UpdateEncoderAngle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_C), UpdateEncoderSpeed, CHANGE);

  nh.initNode();
  nh.loginfo("Arduino connected to ROS");

  nh.subscribe(AGVsubscriber);
  nh.subscribe(BuzzerSubscriber);

  nh.advertise(VelocityX);
  nh.advertise(VelocityY);
  nh.advertise(VelocityZ);
  nh.advertise(CheckSpeed);
  nh.advertise(CheckAngle);
  nh.advertise(EncSpeed);
  nh.advertise(EncAngle);
  nh.advertise(Emergen);
  nh.advertise(Success);

}

void loop() {

  LinearX_msg.data = speed;
  LinearY_msg.data = 0;
  AngularZ_msg.data = (speed * tan(CurrentSteeringAngle * (PI / 180))) / wheelBase;

  Speed_msg.data = speed;
  Angle_msg.data = CurrentSteeringAngle;

  EncSpeed_msg.data = CounterEncoderSpeed;
  EncAngle_msg.data = CounterEncoderAngle;

  if (angularVelocityZ == 0) {
    TargetSteeringAngle_deg = 0;
    TargetSpeed = linearVelocityX;
  } else {
    turningRadius = linearVelocityX / angularVelocityZ;
    TargetSteeringAngle_rad = atan(wheelBase / turningRadius);
    TargetSteeringAngle_deg = constrain(degrees(TargetSteeringAngle_rad), -70, 70);
    TargetSpeed = (abs(angularVelocityZ) * wheelBase) / abs(tan(TargetSteeringAngle_deg * (PI / 180)));
  }

  TargetCount = ((TargetSpeed * PPR) / (wheelRadius * 2.0 * PI * 10));

  currentTime = millis();
  deltaTime = currentTime - lastTime;
  lastTime = currentTime;

  if (deltaTime >= 100) {
    UpdateCurrentSpeed();
    CounterEncoderSpeed = 0;
  }

  UpdateCurrentAngle();
  PWMmotor = PIDspeed.Evaluate(CounterEncoderSpeed, TargetCount, deltaTime / 1000.0);

  switch (state) {
    case setDirection:
      if (digitalRead(LIMIT_SWITCH_PIN_LEFT) == HIGH) {
        analogWrite(Motor_R, 0);
        analogWrite(Motor_L, 0);
        CounterEncoderAngle = 0;
        state = onLoop;
      } else {
        analogWrite(Motor_R, 0);
        analogWrite(Motor_L, PWMsteering);
      }
    break;

    case onLoop:

      if (digitalRead(EMERGENCY_SWITCH_PIN) == LOW) { 
        Emergen_msg.data = "on"; 

        analogWrite(Motor_R, 0);
        analogWrite(Motor_L, 0);
        analogWrite(Motor_F, 0);
        analogWrite(Motor_B, 0);
      
      } else {
        Emergen_msg.data = "off"; 

        SteeringAngleError = TargetSteeringAngle_deg - CurrentSteeringAngle;
        if ( SteeringAngleError < 0.0) {
          analogWrite(Motor_R, PWMsteering);       // TurnRight
          analogWrite(Motor_L, 0);
        } else if ( SteeringAngleError > 0.0){
          analogWrite(Motor_R, 0);                 // TurnLeft
          analogWrite(Motor_L, PWMsteering);
        } else { 
          analogWrite(Motor_R, 0);                 // StopTurn
          analogWrite(Motor_L, 0);
        }

        if (linearVelocityX > 0) {
          analogWrite(Motor_F, PWMmotor);          // Forward
          analogWrite(Motor_B, 0);
        } else if (linearVelocityX < 0) {
          analogWrite(Motor_F, 0);                 // Backward
          analogWrite(Motor_B, PWMmotor);
        } else {
          analogWrite(Motor_F, 0);                 // Stop
          analogWrite(Motor_B, 0);
        }
      }

      if (digitalRead(SUCCESS_WORK_PIN) == LOW) { 
        Success_msg.data = "on"; 
      } else { 
        Success_msg.data = "off"; 
      }
    break;

  }

  VelocityX.publish(&LinearX_msg);
  VelocityY.publish(&LinearY_msg);
  VelocityZ.publish(&AngularZ_msg);
  CheckSpeed.publish(&Speed_msg);
  CheckAngle.publish(&Angle_msg);
  EncSpeed.publish(&EncSpeed_msg);
  EncAngle.publish(&EncAngle_msg);
  Emergen.publish(&Emergen_msg);
  Success.publish(&Success_msg);

  nh.spinOnce();
  delay(10);
}