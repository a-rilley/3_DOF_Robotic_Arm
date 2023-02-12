#include <PID_v1.h>

// L298N + DC Motors.
#define motor1En 5
#define motor1CW 7
#define motor1CCW 6

#define motor2En 8
#define motor2CW 10
#define motor2CCW 9

#define motor3En 44
#define motor3CW 46
#define motor3CCW 45

// Interrupt pins.
#define encoderA1 2   // Encoder A motor 1
#define encoderB1 3   // Encoder B motor 1
#define encoderA2 18  // Encoder A motor 2
#define encoderB2 19  // Encoder B motor 2
#define encoderA3 20  // Encoder A motor 3
#define encoderB3 21  // Encoder B motor 3

// A3144 hall-effect sensors (for homing arms).
#define hallSens1 A0
#define hallSens2 A1
#define hallSens3 A2

// PID.
double kp1 = 3.5, ki1 = 0.000000001, kd1 = 0.4; // Adjust to tune PID control for motor 1.
double kp2 = 3.5, ki2 = 0.000000001, kd2 = 0.4; // Adjust to tune PID control for motor 2.
double kp3 = 3.5, ki3 = 0.000000001, kd3 = 0.4; // Adjust to tune PID control for motor 3.
double input1 = 0, output1 = 0, setpoint1 = 0;
double input2 = 0, output2 = 0, setpoint2 = 0;
double input3 = 0, output3 = 0, setpoint3 = 0;

PID m1PID(&input1, &output1, &setpoint1, kp1, ki1, kd1, DIRECT);
PID m2PID(&input2, &output2, &setpoint2, kp2, ki2, kd2, DIRECT);
PID m3PID(&input3, &output3, &setpoint3, kp3, ki3, kd3, DIRECT);

// Motor.
volatile long encoderValue1 = 0;
volatile long encoderValue2 = 0;
volatile long encoderValue3 = 0;
int angle = 0;
int PPR1 = 1929;   // Pulses per revolution - motor 1.

double theta1 = 5;
double theta2 = 1;
double theta3 = 5;

int reqEV1 = 0;   // Required encoder value 1 (SETPOINT).
int reqEV2 = 0;   // Required encoder value 2 (SETPOINT).
int reqEV3 = 0;   // Required encoder value 3 (SETPOINT).

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

  // Encoder interrupts.
  attachInterrupt(digitalPinToInterrupt(encoderA1), encoderEvent1, HIGH); // Position.
  attachInterrupt(digitalPinToInterrupt(encoderA2), encoderEvent2, HIGH);
  attachInterrupt(digitalPinToInterrupt(encoderA3), encoderEvent3, HIGH);

  // PID Motor 1.
  m1PID.SetMode(AUTOMATIC);
  m1PID.SetSampleTime(1);  // Refresh rate of PID controller.
  m1PID.SetOutputLimits(-125,125);  // Max/min PWM values to move motor. Speed of motor.

  // PID Motor 2.
  m2PID.SetMode(AUTOMATIC);
  m2PID.SetSampleTime(1);  // Refresh rate of PID controller.
  m2PID.SetOutputLimits(-125, 125);  // Max/min PWM values to move motor. Speed of motor.

  // PID Motor 3.
  m3PID.SetMode(AUTOMATIC);
  m3PID.SetSampleTime(1);  // Refresh rate of PID controller.
  m3PID.SetOutputLimits(-125, 125);  // Max PWM values to move motor. Speed of motor.

  homing1();
  homing2();
  homing3();
  
  Serial.begin(115200);
}

void loop() {
  // Convert angle to pulse
  reqEV1 = map(theta1, 0, 360, 0, 5800);
  reqEV2 = map(theta2, 0, 360, 0, 5800);
  reqEV3 = map(theta3, 0, 360, 0, 5800);

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

// Encoder events.
void encoderEvent1() {
  if (digitalRead(encoderA1) == digitalRead(encoderB1)) {
    encoderValue1--;
  }
  else {
    encoderValue1++;
  }
}

void encoderEvent2() {
  if (digitalRead(encoderA2) == digitalRead(encoderB2)) {
    encoderValue2--;
  }
  else {
    encoderValue2++;
  }
}

void encoderEvent3() {
  if (digitalRead(encoderA3) == digitalRead(encoderB3)) {
    encoderValue3--;
  }
  else {
    encoderValue3++;
  }
}

// Home the positions of the arms.
void homing1() {
  do {
    digitalWrite(motor1CW, LOW);
    analogWrite(motor1CCW, 64);
    digitalWrite(motor1En, HIGH);
  } while (analogRead(hallSens1) >= 18);
  brake1();
  encoderValue1 = map(8, 0, 360, 0, 5800); // Homing offset.
}
void homing2() {
  do {
    digitalWrite(motor2CW, LOW);
    analogWrite(motor2CCW, 64);
    digitalWrite(motor2En, HIGH);
  } while (analogRead(hallSens2) >= 18);
  brake2();
  encoderValue2 = map(15, 0, 360, 0, 5800); // Homing offset.
}
void homing3() {
  do {
    digitalWrite(motor3CW, LOW);
    analogWrite(motor3CCW, 64);
    digitalWrite(motor3En, HIGH);
  } while (analogRead(hallSens3) >= 18);
  brake3();
  delay(750);
  encoderValue3 = map(0, 0, 360, 0, 5800); // Homing offset.
}

// Motor 1 actions.
void forward1() {
  digitalWrite(motor1CW, HIGH);
  digitalWrite(motor1CCW, LOW);
}

void reverse1() {
  digitalWrite(motor1CW, LOW);
  digitalWrite(motor1CCW, HIGH);
}

void brake1() {
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

void brake2() {
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

void brake3() {
  digitalWrite(motor3CW, LOW);
  digitalWrite(motor3CCW, LOW);
}
