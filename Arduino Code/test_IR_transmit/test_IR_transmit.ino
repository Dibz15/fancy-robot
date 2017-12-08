

int binPin = 7;


void setup() {
 pinMode(binPin, OUTPUT);
}


void loop() {
  digitalWrite(binPin, HIGH);
  delayMicroseconds(516);
  digitalWrite(binPin, LOW);
  delayMicroseconds(516);  
}
