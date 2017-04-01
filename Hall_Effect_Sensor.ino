int magPin = A0; //analog output pin
int magData;  //analog output data
int activationLED = 8; //LED used as flag
void setup() {
  pinMode(magPin, INPUT);
  pinMode(activationLED, OUTPUT);
  digitalWrite(activationLED, LOW);
  Serial.begin(300); //low baud rate so I could follow it
}

void loop() {
  magData= analogRead(magPin); //assigning the data from the pin to the variable
  if ( 510 > magData || magData > 521) //range to set off flag
      digitalWrite(activationLED, HIGH); 
  else 
      digitalWrite(activationLED,LOW);
  Serial.println(magData);
}
/*
int magPin = A0; //analog output pin
int magData;  //analog output data  
void setup() {
  pinMode(magPin, INPUT);
}
int Hallflag()
{
    magData= analogRead(magPin); //assigning the data from the pin to the variable
  if ( 510 > magData || magData > 521) //range to set off flag
     magflag = 1;
  else 
     magflag = 0; 
}
*/

