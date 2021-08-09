const byte ledPin = 13;
const byte interruptPin = 11;
volatile byte state = LOW;
int count;
void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);
}

void loop() {
  digitalWrite(ledPin, state);
  delay(250);
  Serial.println(count);
}

void blink() {
  state = !state;
  count=count+1;
}
