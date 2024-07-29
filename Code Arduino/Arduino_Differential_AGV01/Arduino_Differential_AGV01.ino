#include <ArduinoHardware.h>
#include <Arduino.h>
#include <string.h>
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
      error = target - fabs(value);
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

  std_msgs::Float32 speedL_msg;
  std_msgs::Float32 speedR_msg;
  ros::Publisher CheckSpeedL("SpeedL", &speedL_msg);
  ros::Publisher CheckSpeedR("SpeedR", &speedR_msg);

  std_msgs::Float32 EncL_msg;
  std_msgs::Float32 EncR_msg;
  ros::Publisher EncL("EncL", &EncL_msg);
  ros::Publisher EncR("EncR", &EncR_msg);

  std_msgs::String Emergen_msg;
  std_msgs::String Success_msg;
  ros::Publisher Emergen("Emergen", &Emergen_msg);
  ros::Publisher Success("Success", &Success_msg);

//--------------------------------------[ AGV02 Constant ]
#define PI 3.14159
const float wheelRadius = 0.063;  // .M
const float wheelBase = 0.37;     // .M Tread
const int PPR = 1950;

//--------------------------------------[ Define Device Pin ]
#define ENCODER_L_PIN_A 2
#define ENCODER_L_PIN_B 3
#define ENCODER_R_PIN_A 18
#define ENCODER_R_PIN_B 19
#define EMERGENCY_SWITCH_PIN 41
#define SUCCESS_WORK_PIN 40
#define BUZZER 46

//--------------------------------------[ Variable : Motor Pin (L, R)]
const int MotorF[] = {6, 4};
const int MotorB[] = {7, 5};

//--------------------------------------[ Variable : PID control]
const float Kp = 1.3;
const float Ki = 0.0;
const float Kd = 0.0;

PIDspeed PIDspeed[2];

//--------------------------------------[ Variable : Encoder ]
volatile long CounterEncoderL = 0;
volatile long CounterEncoderR = 0;

//--------------------------------------[ Variable : Speed ]
double linearVelocityX;
double linearVelocityY;
double angularVelocityZ;
double TargetSpeed[] = {0, 0};
double TargetSpeedAvg;
double TargetCount;
double speedL;
double speedR;
int PWMmotorL;
int PWMmotorR;

//--------------------------------------[ Variable : Time ]
unsigned long lastTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime;

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

void UpdateEncoderSpeed_L() {
  CounterEncoderL += (digitalRead(ENCODER_L_PIN_A) == digitalRead(ENCODER_L_PIN_B)) ? -1 : 1;
}

void UpdateEncoderSpeed_R() {
  CounterEncoderR += (digitalRead(ENCODER_R_PIN_A) == digitalRead(ENCODER_R_PIN_B)) ? -1 : 1;
}

void UpdateCurrentSpeed() {
  double RPS_L = static_cast<double>(CounterEncoderL * 10) / PPR;
  speedL = wheelRadius * 2.0 * PI * RPS_L;

  double RPS_R = static_cast<double>(CounterEncoderR * 10) / PPR;
  speedR = wheelRadius * 2.0 * PI * RPS_R;
}

void setup() {
  Serial.begin(57600);
  
  PIDspeed[0].SetParams(Kp, Ki, Kd);
  PIDspeed[1].SetParams(Kp, Ki, Kd);

  pinMode(MotorF[0], OUTPUT);
  pinMode(MotorB[0], OUTPUT);
  pinMode(MotorF[1], OUTPUT);
  pinMode(MotorB[1], OUTPUT);

  pinMode(ENCODER_L_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_L_PIN_B, INPUT_PULLUP);
  pinMode(ENCODER_R_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_R_PIN_B, INPUT_PULLUP);

  pinMode(EMERGENCY_SWITCH_PIN, INPUT_PULLUP); 
  pinMode(SUCCESS_WORK_PIN, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_PIN_A), UpdateEncoderSpeed_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_PIN_A), UpdateEncoderSpeed_R, CHANGE);

  nh.initNode();
  nh.loginfo("Arduino connected to ROS");

  nh.subscribe(AGVsubscriber);
  nh.subscribe(BuzzerSubscriber);

  nh.advertise(VelocityX);
  nh.advertise(VelocityY);
  nh.advertise(VelocityZ);
  nh.advertise(CheckSpeedL);
  nh.advertise(CheckSpeedR);
  nh.advertise(EncL);
  nh.advertise(EncR);
  nh.advertise(Emergen);
  nh.advertise(Success);
}

void loop() {

  LinearX_msg.data = (speedR + speedL) * 0.5;
  LinearY_msg.data = 0;
  AngularZ_msg.data = (speedR - speedL) / wheelBase;

  speedL_msg.data = speedL;
  speedR_msg.data = speedR;

  EncL_msg.data = CounterEncoderL;
  EncR_msg.data = CounterEncoderR;

  TargetSpeed[0] = linearVelocityX  - ((angularVelocityZ * wheelBase) / 2);
  TargetSpeed[1] = linearVelocityX  + ((angularVelocityZ * wheelBase) / 2);
  TargetSpeedAvg = (abs(TargetSpeed[0]) + abs(TargetSpeed[1])) / 2;
  TargetCount = (TargetSpeedAvg * PPR) / (wheelRadius * 2.0 * PI * 10);

  currentTime = millis();
  deltaTime = currentTime - lastTime;

  if (deltaTime >= 100) {
    UpdateCurrentSpeed();
    CounterEncoderL = 0;
    CounterEncoderR = 0;
    lastTime = currentTime;
  }

  PWMmotorL = PIDspeed[0].Evaluate(CounterEncoderL, TargetCount, deltaTime / 1000.0);
  PWMmotorR = PIDspeed[1].Evaluate(CounterEncoderR, TargetCount, deltaTime / 1000.0);

  if (digitalRead(EMERGENCY_SWITCH_PIN) == LOW) { 
    Emergen_msg.data = "on"; 

    analogWrite(MotorF[0], 0);
    analogWrite(MotorB[0], 0);
    analogWrite(MotorF[1], 0);
    analogWrite(MotorB[1], 0);
  
  } else { 
    Emergen_msg.data = "off"; 
    
    if (TargetSpeed[0] > 0.0) {
      analogWrite(MotorF[0], PWMmotorL);
      analogWrite(MotorB[0], 0);
    } else if (TargetSpeed[0] < 0.0) {
      analogWrite(MotorF[0], 0);
      analogWrite(MotorB[0], PWMmotorL);
    } else {
      analogWrite(MotorF[0], 0);
      analogWrite(MotorB[0], 0);
    }

    if (TargetSpeed[1] > 0.0) {
      analogWrite(MotorF[1], PWMmotorR);
      analogWrite(MotorB[1], 0);
    } else if (TargetSpeed[1] < 0.0) {
      analogWrite(MotorF[1], 0);
      analogWrite(MotorB[1], PWMmotorR);
    } else {
      analogWrite(MotorF[1], 0);
      analogWrite(MotorB[1], 0);
    }
  }

  if (digitalRead(SUCCESS_WORK_PIN) == LOW) { 
    Success_msg.data = "on"; 
  } else { 
    Success_msg.data = "off"; 
  }

  VelocityX.publish(&LinearX_msg);
  VelocityY.publish(&LinearY_msg);
  VelocityZ.publish(&AngularZ_msg);
  CheckSpeedL.publish(&speedL_msg);
  CheckSpeedR.publish(&speedR_msg);
  EncL.publish(&EncL_msg);
  EncR.publish(&EncR_msg);
  Emergen.publish(&Emergen_msg);
  Success.publish(&Success_msg);

  nh.spinOnce();
  delay(100);
}
