//IR Receiver.
//Variable decleration

byte input;    //the input from the dig pins 
int ir[5];  
int degree[5] = {0,45,90,135,180};
int zero =0;       //to store how many 1 we are reading
int sum =0;       //sum for the sum of degeees.

void setup() {
  // put your setup code here, to run once:
  DDRB = 0x0;
  Serial.begin(9600);
}

void loop() {
  zero=0;
  sum=0;
  for(int i=0 ; i<5 ; i++){
    ir[i] =0 ;
  }

  //Serial.println(i);
  for (int j =0 ; j < 80; j++)
  {
    input = PINB;
    delay(1);
    //Serial.print(bitRead(input,i));
    if (bitRead(input, 0) == 0)
      ir[0] = 1;
    if (bitRead(input, 1) == 0)
      ir[1] = 1;
    if (bitRead(input, 2) == 0)
      ir[2] = 1;
    if (bitRead(input, 3) == 0)
      ir[3] = 1;
    if (bitRead(input, 4) == 0)
      ir[4] = 1;
    
  }

 for (int i=0 ;i <5; i++){
   //Serial.println(bitRead(input,i));
  if (ir[i]== 1)
  zero++;
 }
 /*for(int i=0 ;i<5; i++){
  Serial.print(ir[i]);
  Serial.print(" ");}
  Serial.println();
  Serial.println(zero);
  delay(1000);
 */
 if (zero == 0)                         //there is no signals.
  Serial.println("200");
  
 else if (zero == 1){                  // Only one IR reciever detects the signal.
    for (int i =0 ; i<5; i++){
      if (ir[i]==1)
          Serial.println(degree[i]);}
    }
 else if (zero == 2){                 // There is two IR receiver detects the signal.
  for (int i= 0; i<5; i++){
    if (ir[i]== 1)
        sum += degree[i];
    }
    Serial.println(sum/2);
 }
 
 else                                 
  Serial.println("300");

 }
