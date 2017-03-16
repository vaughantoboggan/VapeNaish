//set global variables
int fore[2] = {8, 9}; //first is trig, second is echo
int aft[2] = {6, 7}; //first is trig, second is echo
int trig = 0, echo = 1;
long foreDuration = 0, aftDuration = 0, direct = 0, pingTimer = 0;;

void setup() {
  //set pin modes for the fore ultrasonic
  pinMode(fore[0], OUTPUT);
  pinMode(fore[1], INPUT);
  //set pin modes for the aft ultrasonic
  pinMode(aft[0], OUTPUT);
  pinMode(aft[1], INPUT);
  // initialize serial communication:
  Serial.begin(9600);
}

bool pingQueue = true;
//this function determines which direction to go
int wallCorrection(long distance = 0){
  double analogDist = distance * 58; //converts centimeters to microsecond value
  if (pingQueue){
    pingTimer = millis();
    digitalWrite(fore[trig], LOW);
    delayMicroseconds(2);
    digitalWrite(fore[trig], HIGH);
    delayMicroseconds(5);
    digitalWrite(fore[trig], LOW);

    digitalWrite(fore[echo], HIGH);
    foreDuration = pulseIn(fore[echo], HIGH);
    pingQueue = false;
  }
  
  if (millis() >= (pingTimer + 50) && !pingQueue){
    digitalWrite(aft[trig], LOW);
    delayMicroseconds(2);
    digitalWrite(aft[trig], HIGH);
    delayMicroseconds(5);
    digitalWrite(aft[trig], LOW);

    digitalWrite(aft[echo], HIGH);
    aftDuration = pulseIn(aft[echo], HIGH);
    pingQueue = true;
  }  
  

  Serial.print("Fore: ");
  Serial.println(foreDuration);
  Serial.print("Aft: ");
  Serial.println(aftDuration);
  if (foreDuration > (analogDist + 15) && aftDuration > (analogDist + 15)) return 3; //if too far away, need to go right to correct
  if (foreDuration < (analogDist - 15) && aftDuration < (analogDist - 15)) return 2; //if too close, need to go left to correct
  if (foreDuration > (aftDuration + 25)) return 3; //needs to go right, can figure out what to return later
  else if (foreDuration < (aftDuration - 25)) return 2; //needs to go left, can figure out what to return later
  else return 1;
}

//no real function, just displaying data
void loop()
{
  direct = wallCorrection(5); //calls on correction function, parameter is distance in cm
  Serial.println(direct);
  delay(100);
}
