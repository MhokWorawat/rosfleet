#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <AFMotor.h>
#include <PinChangeInterrupt.h>

//--------------------------------------[ ROS ]
ros::NodeHandle nh;
std_msgs::Float32MultiArray speed_msg;
ros::Publisher speed_pub("speed", &speed_msg);

//--------------------------------------[ AGV04 Constant ]
#define PI 3.14159
const float wheelRadius = 0.028;      // meters
const float wheelBaseWidth = 0.225;   // meters
const float wheelBaseLength = 0.191;  // meters
const int PPR = 1250; // Pulses per Revolution

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
AF_DCMotor motor1(3);  // Motor 1
AF_DCMotor motor2(2);  // Motor 2
AF_DCMotor motor3(4);  // Motor 3
AF_DCMotor motor4(1);  // Motor 4

//--------------------------------------[ Variable : Encoder ]
volatile long CounterEncoder1 = 0;
volatile long CounterEncoder2 = 0;
volatile long CounterEncoder3 = 0;
volatile long CounterEncoder4 = 0;

//--------------------------------------[ Variable : Speed ]
double speed_motor1 = 0.0;
double speed_motor2 = 0.0;
double speed_motor3 = 0.0;
double speed_motor4 = 0.0;

//--------------------------------------[ Variable : Time ]
unsigned long currentTime = 0;
unsigned long lastTime = 0;
unsigned long deltaTime = 0;
const int loopTime = 100; // Loop time in milliseconds

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
  speed_motor1 = wheelRadius * 2.0 * PI * RPS_Motor1;

  double RPS_Motor2 = (CounterEncoder2 * (1000 / loopTime)) / PPR;
  speed_motor2 = wheelRadius * 2.0 * PI * RPS_Motor2;

  double RPS_Motor3 = (CounterEncoder3 * (1000 / loopTime)) / PPR;
  speed_motor3 = wheelRadius * 2.0 * PI * RPS_Motor3;

  double RPS_Motor4 = (CounterEncoder4 * (1000 / loopTime)) / PPR;
  speed_motor4 = wheelRadius * 2.0 * PI * RPS_Motor4;
}

void publisher() {
  // Prepare message
  speed_msg.data_length = 4;
  speed_msg.data[0] = speed_motor1;
  speed_msg.data[1] = speed_motor2;
  speed_msg.data[2] = speed_motor3;
  speed_msg.data[3] = speed_motor4;
  
  // Publish the message
  speed_pub.publish(&speed_msg);
}

void setup() {
  Serial.begin(115200);
  nh.initNode();
  nh.advertise(speed_pub);

  pinMode(encoderPin1A, INPUT_PULLUP);
  pinMode(encoderPin1B, INPUT_PULLUP);
  pinMode(encoderPin2A, INPUT_PULLUP);
  pinMode(encoderPin2B, INPUT_PULLUP);
  pinMode(encoderPin3A, INPUT_PULLUP);
  pinMode(encoderPin3B, INPUT_PULLUP);
  pinMode(encoderPin4A, INPUT_PULLUP);
  pinMode(encoderPin4B, INPUT_PULLUP);

  attachPCINT(digitalPinToPCINT(encoderPin1A), UpdateEncoder1, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPin2A), UpdateEncoder2, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPin3A), UpdateEncoder3, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPin4A), UpdateEncoder4, CHANGE);
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
    lastTime = currentTime;

    // Publish speed data
    publisher();
  }

  int PWM = 0; // Placeholder for PWM value
  motor1.setSpeed(PWM);
  motor1.run(FORWARD);
  motor2.setSpeed(PWM);
  motor2.run(FORWARD);
  motor3.setSpeed(PWM);
  motor3.run(FORWARD);
  motor4.setSpeed(PWM);
  motor4.run(FORWARD);

  // Print data to serial monitor
  Serial.print("PWM : ");
  Serial.print(PWM);
  Serial.print(" Speed M1 : ");
  Serial.print(speed_motor1, 3);
  Serial.print(" Speed M2 : ");
  Serial.print(speed_motor2, 3);
  Serial.print(" Speed M3 : ");
  Serial.print(speed_motor3, 3);
  Serial.print(" Speed M4 : ");
  Serial.println(speed_motor4, 3);

  nh.spinOnce();
  delay(10); // Small delay to avoid blocking
}