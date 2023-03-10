/* METE-3100U Actuators and Power Electronics
   3-DOF Robot Arm Design Project
*/

#include <PID_v1.h>
#include <Servo.h>
#include <math.h>

// L298N + DC Motors.
#define motor1En 5
#define motor1CW 6 
#define motor1CCW 7

#define motor2En 8
#define motor2CW 9
#define motor2CCW 10

#define motor3En 44
#define motor3CW 45
#define motor3CCW 46

// Interrupt pins.
#define encoderA1 2   // Encoder A motor 1
#define encoderB1 3   // Encoder B motor 1
#define encoderA2 18  // Encoder A motor 2
#define encoderB2 19  // Encoder B motor 2
#define encoderA3 20  // Encoder A motor 3
#define encoderB3 21  // Encoder B motor 3

// PID.
double kp1 = 5, ki1 = 1, kd1 = 0.01; // Adjust to tune PID control for motor 1.
double kp2 = 5, ki2 = 1, kd2 = 0.01; // Adjust to tune PID control for motor 2.
double kp3 = 5, ki3 = 1, kd3 = 0.01; // Adjust to tune PID control for motor 3.
double input1 = 0, output1 = 0, setpoint1 = 0;
double input2 = 0, output2 = 0, setpoint2 = 0;
double input3 = 0, output3 = 0, setpoint3 = 0;

PID m1PID(&input1, &output1, &setpoint1, kp1, ki1, kd1, DIRECT);
PID m2PID(&input2, &output2, &setpoint2, kp2, ki2, kd2, DIRECT);
PID m3PID(&input3, &output3, &setpoint3, kp2, ki3, kd3, DIRECT);

// Motor.
volatile long encoderValue1 = 0;
volatile long encoderValue2 = 0;
volatile long encoderValue3 = 0;
int angle = 360;
int PPR1 = xxx;   // Pulses per revolution - motor 1.
int PPR2 = xxx;   // Pulses per revolution - motor 2.
int PPR3 = xxx;   // Pulses per revolution - motor 3.
int reqEV1 = 0;   // Required encoder value 1 (SETPOINT).
int reqEV2 = 0;   // Required encoder value 2 (SETPOINT).
int reqEV3 = 0;   // Required encoder value 3 (SETPOINT).

// Gripper.
Servo gripperServo;
gripperAngle = 90; // Open: 90. Closed: 0.

// Fwd & Inv Kinematics.
double x = 10.0;
double y = 10.0;
double z = 10.0;

double L1 = xxx; // L1 = 200 mm.
double L2 = xxx; // L2 = 150 mm.
double L3 = xxx; // L3 = 40 mm.
double theta1, theta2, theta3;


void setup() {

  // Base motor.
  pinMode(motor1En, OUTPUT);
  pinMode(motor1CW, OUTPUT);
  pinMode(motor1CCW, OUTPUT);

  // Arm 1 motor.
  pinMode(motor2En, OUTPUT);
  pinMode(motor2CW, OUTPUT);
  pinMode(motor2CCW, OUTPUT);

  // Arm 2 motor
  pinMode(motor3En, OUTPUT);
  pinMode(motor3CW, OUTPUT);
  pinMode(motor3CCW, OUTPUT);

  // Encoders.
  pinMode(encoderA1, INPUT_PULLUP);  // Base.
  pinMode(encoderA2, INPUT_PULLUP);  // Arm 1.
  pinMode(encoderA3, INPUT_PULLUP);  // Arm 2.
  pinMode(encoderB1, INPUT_PULLUP);
  pinMode(encoderB2, INPUT_PULLUP);
  pinMode(encoderB3, INPUT_PULLUP);


  // Turn on pullup resistors.   ** May not be necessary.
  digitalWrite(encoderA1, HIGH);
  digitalWrite(encoderA2, HIGH);
  digitalWrite(encoderA3, HIGH);
  digitalWrite(encoderB1, HIGH);
  digitalWrite(encoderB2, HIGH);
  digitalWrite(encoderB3, HIGH);

  // Encoder interrupts.
  attachInterrupt(digitalPinToInterrupt(encoderA1), encoderEvent1, HIGH); // Position.
  attachInterrupt(digitalPinToInterrupt(encoderA2), encoderEvent2, HIGH);
  attachInterrupt(digitalPinToInterrupt(encoderA3), encoderEvent3, HIGH);


  Serial.begin(9600);

  // Set 31 kHz PWM to prevent motor whine.
  TCCR1B = TCCR1B & 0b11111000 | 1;

  // Gripper.
  gripperServo.attach(A0, 600, 2500);    ///*******CONFIRM VALUES.
  // initial servo value - open gripper
  gripperServo.write(90);
  delay(1000);


  // PID Motor 1.
  m1PID.SetMode(AUTOMATIC);
  m1PID.SetSampleTime(1);  // Refresh rate of PID controller.
  m1PID.SetOutputLimits(-125, 125);  // Max/min PWM values to move motor. Speed of motor.

  // PID Motor 2.
  m2PID.SetMode(AUTOMATIC);
  m2PID.SetSampleTime(1);  // Refresh rate of PID controller.
  m2PID.SetOutputLimits(-125, 125);  // Max/min PWM values to move motor. Speed of motor.

  // PID Motor 3.
  m3PID.SetMode(AUTOMATIC);
  m3PID.SetSampleTime(1);  // Refresh rate of PID controller.
  m3PID.SetOutputLimits(-125, 125);  // Max PWM values to move motor. Speed of motor.
}

void loop() {

  inverseKinematics();

  // Convert angle to pulse
  reqEV1 = map(theta1, 0, 360, 0, PPR1);
  reqEV2 = map(theta2, 0, 360, 0, PPR2);
  reqEV3 = map(theta3, 0, 360, 0, PPR3);

  setpoint1 = reqEV1;
  input1 = encoderValue1;
  m1PID.Compute();
  pwmOut1(output1);

  setpoint2 = reqEV2;
  input2 = encoderValue2;
  m2PID.Compute();
  pwmOut2(output2);

  setpoint3 = reqEV3;
  input3 = encoderValue3;
  m3PID.Compute();
  pwmOut3(output3);
}


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////   FUNCTIONS    ////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////



// Kinematics formulas.

void forwardKinematics() {
  float theta1F = theta1 * PI / 180;   // degrees to radians
  float theta2F = theta2 * PI / 180;
  float theta3F = theta3 * PI / 180;
  float xP = round((L2 * cos(theta2F) + L3 * cos(theta3F - theta3F)) * cos(theta1F));
  float yP = round((L2 * cos(theta2F) + L3 * cos(theta3F - theta3F)) * sin(theta1F));
  float zP = round(L1 + L2 * sin(theta2F) - L3 * sin(theta3F - theta2F));
}

void inverseKinematics(float x, float y, float z) {

  float c3 = (pow(x, 2) + pow(y, 2) + pow((z - L1), 2) - pow(L2, 2) - pow(L3, 2)) / (2 * L2 * L3);
  float s3 = sqrt(1 - pow(c3, 2));

  theta1 = atan2(y, x);
  theta2 = atan2(z - L1, sqrt(pow(x, 2) + pow(y, 2)));
  theta3 = atan2(abs(sqrt(s3, c3)));

  theta1 = theta1 * 180 / PI;
  theta2 = theta2 * 180 / PI;
  theta3 = theta3 * 180 / PI;

  theta1 = round(theta1);
  theta2 = round(theta2);
  theta3 = round(theta3);
}

////////////////////////////////////////////////////////////////////////

// PWM output for motor 1.
void pwmOut1(int out) {
  if (out > 0) {                 // If reqEV > encoderValue motor will move in forward direction.
    analogWrite(motor1En, out);   // Enable motor to reach desired angle.
    forward1();
  }
  else {
    analogWrite(motor1En, abs(out));  // If reqEV < encoderValue motor will move in reverse direction.
    reverse1();
  }
}

// PWM output for motor 2.
void pwmOut2(int out) {
  if (out > 0) {                 // If reqEV > encoderValue motor will move in forward direction.
    analogWrite(motor2En, out);   // Enable motor to reach desired angle.
    forward2();
  }
  else {
    analogWrite(motor2En, abs(out));  // If reqEV < encoderValue motor will move in reverse direction.
    reverse2();
  }
}

// PWM output for motor 3.
void pwmOut3(int out) {
  if (out > 0) {                 // If reqEV > encoderValue motor will move in forward direction.
    analogWrite(motor3En, out);   // Enable motor to reach desired angle.
    forward3();
  }
  else {
    analogWrite(motor3En, abs(out));  // If reqEV < encoderValue motor will move in reverse direction.
    reverse3();
  }
}

////////////////////////////////////////////////////////////////////////

// Encoder events.
void encoderEvent1() {
  if (digitalRead(encoderA1) == digitalRead(encoderB1)) {
    encoderValue1++;
  }
  else {
    encoderValue1--;
  }
}

void encoderEvent2() {
  if (digitalRead(encoderA2) == digitalRead(encoderB2)) {
    encoderValue2++;
  }
  else {
    encoderValue2--;
  }
}

void encoderEvent3() {
  if (digitalRead(encoderA3) == digitalRead(encoderB3)) {
    encoderValue3++;
  }
  else {
    encoderValue3--;
  }
}

////////////////////////////////////////////////////////////////////////

// Motor 1 actions.
void forward1() {
  digitalWrite(motor1CW, HIGH);
  digitalWrite(motor1CCW, LOW);
}

void reverse1() {
  digitalWrite(motor1CW, LOW);
  digitalWrite(motor1CCW, HIGH);
}

void halt1() {
  digitalWrite(motor1CW, LOW);
  digitalWrite(motor1CCW, LOW);
}

// Motor 2 actions.
void forward2() {
  digitalWrite(motor2CW, HIGH);
  digitalWrite(motor2CCW, LOW);
}

void reverse2() {
  digitalWrite(motor2CW, LOW);
  digitalWrite(motor2CCW, HIGH);
}

void halt2() {
  digitalWrite(motor2CW, LOW);
  digitalWrite(motor2CCW, LOW);
}

// Motor 3 actions.
void forward3() {
  digitalWrite(motor3CW, HIGH);
  digitalWrite(motor3CCW, LOW);
}

void reverse3() {
  digitalWrite(motor3CW, LOW);
  digitalWrite(motor3CCW, HIGH);
}

void halt3() {
  digitalWrite(motor3CW, LOW);
  digitalWrite(motor3CCW, LOW);
}
