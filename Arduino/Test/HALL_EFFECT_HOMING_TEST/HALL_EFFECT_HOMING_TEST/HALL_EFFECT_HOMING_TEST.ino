#define ledPin 13
#define intPin A0
volatile byte state = LOW;

void setup() {
  pinMode(ledPin, OUTPUT);

  Serial.begin(9600);

}

void loop() {
  blink();
  digitalWrite(ledPin, state);
  delay(1);
}

void blink() {
  if (analogRead(intPin) >= 500) {
    state = HIGH;
    Serial.println("Detected.");
    Serial.println(analogRead(intPin));
  }
  else {
    state = LOW;  // analogRead = 18-20 based on distance from sensor.
    Serial.println(analogRead(intPin));
  }
}
