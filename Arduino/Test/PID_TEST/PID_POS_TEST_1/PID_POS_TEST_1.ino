#include <PID_v1.h>

// L298N + DC Motors.
#define motor1En 5
#define motor1CW 7    // IN3.
#define motor1CCW 6   // IN4.

#define encoderA1 2   // Encoder A motor 1.
#define encoderB1 3   // Encoder B motor 1.

double kp1 = 2, ki1 = 0.5, kd1 = 0.2; // Adjust to tune PID control for motor 1.
double input1 = 0, output1 = 0, setpoint1 = 0;
PID m1PID(&input1, &output1, &setpoint1, kp1, ki1, kd1, DIRECT);

// Motor.
volatile long encoderValue1 = 0;
int angle = 0;
int PPR1 = 1929;   // Pulses per revolution - motor 1.

double theta1 = 360;

int reqEV1 = 0;   // Required encoder value 1 (SETPOINT).



void setup() {
  // Base motor.
  pinMode(motor1En, OUTPUT);
  pinMode(motor1CW, OUTPUT);
  pinMode(motor1CCW, OUTPUT);
  pinMode(encoderA1, INPUT_PULLUP);  // Base.
  pinMode(encoderB1, INPUT_PULLUP);
  
  // Encoder interrupts.
  attachInterrupt(digitalPinToInterrupt(encoderA1), encoderEvent1, HIGH); // Position.


  m1PID.SetMode(AUTOMATIC);
  m1PID.SetSampleTime(1);  // Refresh rate of PID controller.
  m1PID.SetOutputLimits(-255, 255);  // Max/min PWM values to move motor. Speed of motor.

  Serial.begin(9600);
}

void loop() {
  // Convert angle to pulse
  reqEV1 = map(theta1, 0, 360, 0, 5860);

  setpoint1 = reqEV1;
  input1 = encoderValue1;
  m1PID.Compute();
  pwmOut1(output1);

  Serial.println(encoderValue1);



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
// Encoder events.
void encoderEvent1() {
  if (digitalRead(encoderA1) == digitalRead(encoderB1)) {
    encoderValue1--;
  }
  else {
    encoderValue1++;
  }
}

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
