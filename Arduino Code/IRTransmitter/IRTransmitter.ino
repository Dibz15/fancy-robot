//This code would generate a modulated 38kHz IR signal
int count;
void setup() {
  // put your setup code here, to run once:

  //pinMode(4, OUTPUT);                          //pin 4 is connected to base of the transistor
  DDRD = 0b00010000;
}

void loop() {
 
  for(int i=0; i < 2000; i++){                //Turn On and OFF for 0.3 seconds
    digitalWrite(4, HIGH);
    //PORTD = 0b00011000;
    delayMicroseconds(11.5);
    digitalWrite(4, LOW);
    //PORTD = 0x0;
    delayMicroseconds(11.5);
  }
  
  digitalWrite(4,LOW);
  delay(50);                //turn off for 0.3 seconds
}
