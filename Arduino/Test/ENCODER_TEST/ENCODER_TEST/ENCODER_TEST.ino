/*  Encoder Test
*/
// 37 rpm @ 12 V, 115 mA
// PPS = 1190 +/- 10
// PPR = (encoderValueA * 60 / rpm) = 1929


// Arduino pin assignment.
#define motor1En 5
#define motor1CW 7    // IN3.
#define motor1CCW 6   // IN4.
#define encoderA 2   // Encoder interrupt pins.
#define encoderB 3

volatile long encoderValueA = 0;
volatile long encoderValueB = 0;


// Interval settings.
int interval = 10000;
long previousMillis = 0;
long currentMillis = 0;

// Revolutions per minute.
int rpm = 0;

void setup() {
  Serial.begin(9600);

  // Declare pinModes.
  pinMode(motor1En, OUTPUT);
  pinMode(motor1CW, OUTPUT);
  pinMode(motor1CCW, OUTPUT);
  pinMode(encoderA, INPUT_PULLUP);  // Base.
  pinMode(encoderB, INPUT_PULLUP);

  digitalWrite(encoderA, HIGH);
  digitalWrite(encoderB, HIGH);

  // Interrupt service routine.
  attachInterrupt(digitalPinToInterrupt(encoderA), pulseA, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderB), pulseB, RISING);

  Serial.println("Start...");
}

void loop() {
  // Update RPM every second.
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    digitalWrite(motor1En, HIGH);
    digitalWrite(motor1CW, HIGH);
    digitalWrite(motor1CCW, LOW);
    
    Serial.println(encoderValueA);
    Serial.println(encoderValueB);

    encoderValueA = 0;
    encoderValueB = 0;
  }
}

void pulseA() {
  encoderValueA++;
}

void pulseB() {
  encoderValueB++;
}
