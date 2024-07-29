#include <ArduinoHardware.h>
#include <PinChangeInterrupt.h>
#include <AFMotor.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

class PIDspeed {
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

  std_msgs::Float32 SpeedWheel1_msg;
  std_msgs::Float32 SpeedWheel2_msg;
  std_msgs::Float32 SpeedWheel3_msg;
  std_msgs::Float32 SpeedWheel4_msg;
  ros::Publisher CheckSpeedWheel1("SpeedWheel1", &SpeedWheel1_msg);
  ros::Publisher CheckSpeedWheel2("SpeedWheel2", &SpeedWheel2_msg);
  ros::Publisher CheckSpeedWheel3("SpeedWheel3", &SpeedWheel3_msg);
  ros::Publisher CheckSpeedWheel4("SpeedWheel4", &SpeedWheel4_msg);

  std_msgs::Float32 EncWheel1_msg;
  std_msgs::Float32 EncWheel2_msg;
  std_msgs::Float32 EncWheel3_msg;
  std_msgs::Float32 EncWheel4_msg;
  ros::Publisher EncWheel1("EncWheel1", &EncWheel1_msg);
  ros::Publisher EncWheel2("EncWheel2", &EncWheel2_msg);
  ros::Publisher EncWheel3("EncWheel3", &EncWheel3_msg);
  ros::Publisher EncWheel4("EncWheel4", &EncWheel4_msg);

  std_msgs::String Emergen_msg;
  std_msgs::String Success_msg;
  ros::Publisher Emergen("Emergen", &Emergen_msg);
  ros::Publisher Success("Success", &Success_msg);

//--------------------------------------[ AGV02 Constant ]
#define PI 3.14159
const float wheelRadius = 0.028;        // .M
const float wheelBaseWidth = 0.225;     // .M
const float wheelBaseLength = 0.191;     // .M
const int PPR = 1250;

//--------------------------------------[ Define Device Pin ]
#define encoderPin1A 53 // FL
#define encoderPin1B 25 // FL
#define encoderPin2A 51 // FR
#define encoderPin2B 23 // FR
#define encoderPin3A 50 // BL
#define encoderPin3B 22 // BL
#define encoderPin4A 52 // BR
#define encoderPin4B 24 // BR
#define EMERGENCY_SWITCH_PIN 49
#define SUCCESS_WORK_PIN 48
#define BUZZER 46

//--------------------------------------[ Variable : Motor Pin (L, R)]
AF_DCMotor motor1(3); //              M1
AF_DCMotor motor2(2); //              M2
AF_DCMotor motor3(4); //              M3
AF_DCMotor motor4(1); //              M4

//--------------------------------------[ Variable : PID control]
const float Kp = 2.5;
const float Ki = 0.0;
const float Kd = 0.0;

PIDspeed PIDspeed[4];

//--------------------------------------[ Variable : Encoder ]
volatile long CounterEncoder1 = 0;
volatile long CounterEncoder2 = 0;
volatile long CounterEncoder3 = 0;
volatile long CounterEncoder4 = 0;

//--------------------------------------[ Variable : Speed ]
double linearVelocityX;
double linearVelocityY;
double angularVelocityZ;
double TargetSpeed;
double TargetSpeedWheel[] = {0,0,0,0};
double TargetCount[] = {0,0,0,0};
double speedWheel[] = {0,0,0,0};
int PWMmotor[] = {0,0,0,0};

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
  double RPS_Motor1 = static_cast<double>(CounterEncoder1 * 10) / PPR;
  speedWheel[0] = wheelRadius * 2.0 * PI * RPS_Motor1;

  double RPS_Motor2 = static_cast<double>(CounterEncoder2 * 10) / PPR;
  speedWheel[1] = wheelRadius * 2.0 * PI * RPS_Motor2;

  double RPS_Motor3 = static_cast<double>(CounterEncoder3 * 10) / PPR;
  speedWheel[2] = wheelRadius * 2.0 * PI * RPS_Motor3;

  double RPS_Motor4 = static_cast<double>(CounterEncoder4 * 10) / PPR;
  speedWheel[3] = wheelRadius * 2.0 * PI * RPS_Motor4;
}

void setup() {
  Serial.begin(57600);

  PIDspeed[0].SetParams(Kp, Ki, Kd);
  PIDspeed[1].SetParams(Kp, Ki, Kd);
  PIDspeed[2].SetParams(Kp, Ki, Kd);
  PIDspeed[3].SetParams(Kp, Ki, Kd);

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

  nh.initNode();
  nh.loginfo("Arduino connected to ROS");

  nh.subscribe(AGVsubscriber);
  nh.subscribe(BuzzerSubscriber);

  nh.advertise(VelocityX);
  nh.advertise(VelocityY);
  nh.advertise(VelocityZ);
  nh.advertise(CheckSpeedWheel1);
  nh.advertise(CheckSpeedWheel2);
  nh.advertise(CheckSpeedWheel3);
  nh.advertise(CheckSpeedWheel4);
  nh.advertise(EncWheel1);
  nh.advertise(EncWheel2);
  nh.advertise(EncWheel3);
  nh.advertise(EncWheel4);
  nh.advertise(Emergen);
  nh.advertise(Success);
}

void loop() {

  LinearX_msg.data = (speedWheel[0] + speedWheel[1] + speedWheel[2] + speedWheel[3]) / 4;
  LinearY_msg.data = (-speedWheel[0] + speedWheel[1] + speedWheel[2] - speedWheel[3]) / 4;
  AngularZ_msg.data = (-speedWheel[0] + speedWheel[1] - speedWheel[2] + speedWheel[3]) / (4 * (wheelBaseWidth + wheelBaseLength));

  SpeedWheel1_msg.data = speedWheel[0];
  SpeedWheel2_msg.data = speedWheel[1];
  SpeedWheel3_msg.data = speedWheel[2];
  SpeedWheel4_msg.data = speedWheel[3];

  EncWheel1_msg.data = CounterEncoder1;
  EncWheel2_msg.data = CounterEncoder2;
  EncWheel3_msg.data = CounterEncoder3;
  EncWheel4_msg.data = CounterEncoder4;

  TargetSpeedWheel[0] = (linearVelocityX - linearVelocityY - ((wheelBaseWidth + wheelBaseLength) * angularVelocityZ));
  TargetSpeedWheel[1] = (linearVelocityX + linearVelocityY + ((wheelBaseWidth + wheelBaseLength) * angularVelocityZ));
  TargetSpeedWheel[2] = (linearVelocityX + linearVelocityY - ((wheelBaseWidth + wheelBaseLength) * angularVelocityZ));
  TargetSpeedWheel[3] = (linearVelocityX - linearVelocityY + ((wheelBaseWidth + wheelBaseLength) * angularVelocityZ));

  TargetCount[0] = ((TargetSpeedWheel[0] * PPR) / (wheelRadius * 2.0 * PI * 10));
  TargetCount[1] = ((TargetSpeedWheel[1] * PPR) / (wheelRadius * 2.0 * PI * 10));
  TargetCount[2] = ((TargetSpeedWheel[2] * PPR) / (wheelRadius * 2.0 * PI * 10));
  TargetCount[3] = ((TargetSpeedWheel[3] * PPR) / (wheelRadius * 2.0 * PI * 10));

  currentTime = millis();
  deltaTime = currentTime - lastTime;

  if (deltaTime >= 100) {
    UpdateCurrentSpeed();
    CounterEncoder1 = 0;
    CounterEncoder2 = 0;
    CounterEncoder3 = 0;
    CounterEncoder4 = 0;
    lastTime = currentTime;
  }

  PWMmotor[0] = PIDspeed[0].Evaluate(CounterEncoder1, TargetCount[0], deltaTime / 1000.0);
  PWMmotor[1] = PIDspeed[1].Evaluate(CounterEncoder2, TargetCount[1], deltaTime / 1000.0);
  PWMmotor[2] = PIDspeed[2].Evaluate(CounterEncoder3, TargetCount[2], deltaTime / 1000.0);
  PWMmotor[3] = PIDspeed[3].Evaluate(CounterEncoder4, TargetCount[3], deltaTime / 1000.0);

  motor1.setSpeed(PWMmotor[0]);
  motor2.setSpeed(PWMmotor[1]);
  motor3.setSpeed(PWMmotor[2]);
  motor4.setSpeed(PWMmotor[3]);
  
  if (digitalRead(EMERGENCY_SWITCH_PIN) == LOW) { 
    Emergen_msg.data = "on"; 

    motor1.run(RELEASE);
    motor2.run(RELEASE);
    motor3.run(RELEASE);
    motor4.run(RELEASE);
  } else { 
    Emergen_msg.data = "off"; 

    if (TargetSpeedWheel[0] > 0.0) {
      motor1.run(FORWARD);
    } else if (TargetSpeedWheel[0] < 0.0) {
      motor1.run(BACKWARD);
    } else {
      motor1.run(RELEASE);
    }

    if (TargetSpeedWheel[1] > 0.0) {
      motor2.run(FORWARD);
    } else if (TargetSpeedWheel[1] < 0.0) {
      motor2.run(BACKWARD);
    } else {
      motor2.run(RELEASE);
    }

    if (TargetSpeedWheel[2] > 0.0) {
      motor3.run(FORWARD);
    } else if (TargetSpeedWheel[2] < 0.0) {
      motor3.run(BACKWARD);
    } else {
      motor3.run(RELEASE);
    }

    if (TargetSpeedWheel[3] > 0.0) {
      motor4.run(FORWARD);
    } else if (TargetSpeedWheel[3] < 0.0) {
      motor4.run(BACKWARD);
    } else {
      motor4.run(RELEASE);
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
  CheckSpeedWheel1.publish(&SpeedWheel1_msg);
  CheckSpeedWheel2.publish(&SpeedWheel2_msg);
  CheckSpeedWheel3.publish(&SpeedWheel3_msg);
  CheckSpeedWheel4.publish(&SpeedWheel4_msg);
  EncWheel1.publish(&EncWheel1_msg);
  EncWheel2.publish(&EncWheel2_msg);
  EncWheel3.publish(&EncWheel3_msg);
  EncWheel4.publish(&EncWheel4_msg);
  Emergen.publish(&Emergen_msg);
  Success.publish(&Success_msg);

  nh.spinOnce();
  delay(10);

}
