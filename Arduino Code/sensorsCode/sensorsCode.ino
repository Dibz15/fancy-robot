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

void findDistance();
void readIR();
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


void loop() {

  distanceAction.check();
  irAction.check();
  batteryAction.check();
  printAction.check();
  
}


void printData() {

  //Print data here
  Serial.print(distance);
  Serial.print(",");
  Serial.print(irAngle);
  Serial.print(",");
  Serial.print(batteryVoltage);
  Serial.println();
  
}


void readIR() {

  //IR sensor data here
  
}

void readBatteryVoltage() {

  //Read battery voltage here
  int raw = analogRead(batteryPin);
  batteryVoltage = (float)raw / 1024.0 * 5.0;

}


void findDistance() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculating the distance
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
