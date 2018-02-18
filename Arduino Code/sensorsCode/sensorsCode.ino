#include "TimedAction/TimedAction.cpp"

// defines pins numbers
const int trigPin = 6;
const int echoPin = 7;
const int ir0 = 1;
const int ir1 = 2;
const int ir2 = 3;
const int ir3 = 4;
const int ir4 = 5;
const int batteryPin = A0;

// defines variables
long duration;

int distance;
float batteryVoltage;
int irAngle;

//Function headers
void findDistance();
void readIR();
long readVcc();
void readBatteryVoltage();
void printData();

//Protothreaded actions
TimedAction distanceAction = TimedAction(200, findDistance);
TimedAction irAction = TimedAction(100, 200, readIR);
TimedAction batteryAction = TimedAction(1000, readBatteryVoltage);
TimedAction printAction = TimedAction(50, 100, printData);


void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  pinMode(ir0, INPUT);  //Infrared reciever pins
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);

  pinMode(batteryPin, INPUT); //Battery voltage read pin

  Serial.begin(9600); // Starts the serial communication


}

//Continuous looping forever!
void loop() {

  distanceAction.check();
  irAction.check();
  batteryAction.check();
  printAction.check();

}

//Print out the serial information.
//This prints in the format distance,irAngle,batteryVoltage\n
void printData() {

  //Print data here
  Serial.print(distance);
  Serial.print(",");
  Serial.print(irAngle);
  Serial.print(",");
  Serial.print(batteryVoltage);
  Serial.println();

}

//Read and calibrate infrared sensors
void readIR() {

  //IR sensor data here

}

//This function reads our Vcc voltage, and returns it in millivolts.
//This is accomplished by reading the internal reference voltage agains AVcc,
//then calibrating Vcc based on the inaccuracy  of that read.
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

void readBatteryVoltage() {

  long Vcc = readVcc();
  //Serial.print("Vcc: ");
  //Serial.println(Vcc);
  //temp
  //Vcc = 4950.0;

  //Read battery voltage here
  //analogReference(INTERNAL);
  unsigned int raw = analogRead(batteryPin);
  batteryVoltage = ((float) raw / 1023.0 * (Vcc / 1000.0));

}

//This function finds the approximate distance of the object
//using the ultrasonic range finder.
//
void findDistance() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  //We put a timeout of 80ms, so it doesn't stall our processor too long.
  duration = pulseIn(echoPin, HIGH, 80000);

  // Calculating the distance
  //This equation comes from the datasheet of the rangefinder.
  distance = duration / 58.0;
}



/*prints val with number of decimal places determine by precision
// precision is a number from 0 to 6 indicating the desired decimial places
example: printDouble( 3.1415, 2); // prints 3.14 (two decimal places)*/
void printDouble( double val, byte precision){
 Serial.print (int(val));  //prints the int part
 if( precision > 0) {
   Serial.print("."); // print the decimal point
   unsigned long frac;
   unsigned long mult = 1;
   byte padding = precision -1;
   while(precision--)
      mult *=10;

   if(val >= 0)
     frac = (val - int(val)) * mult;
   else
     frac = (int(val)- val ) * mult;
   unsigned long frac1 = frac;
   while( frac1 /= 10 )
     padding--;
   while(  padding--)
     Serial.print("0");
   Serial.print(frac,DEC) ;
 }
}
