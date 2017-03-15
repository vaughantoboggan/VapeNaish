//set global variables
int fore[2] = {8, 9}; //first is trig, second is echo
int aft[2] = {6, 7}; //first is trig, second is echo
int trig = 0, echo = 1;
long foreDuration = 0, aftDuration = 0, direct = 0;

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

//this function determines which direction to go
int wallCorrection(){
  digitalWrite(fore[trig], LOW);
  delayMicroseconds(2);
  digitalWrite(fore[trig], HIGH);
  delayMicroseconds(5);
  digitalWrite(fore[trig], LOW);

  digitalWrite(fore[echo], HIGH);
  foreDuration = pulseIn(fore[echo], HIGH);

  delay(50);
  
  digitalWrite(aft[trig], LOW);
  delayMicroseconds(2);
  digitalWrite(aft[trig], HIGH);
  delayMicroseconds(5);
  digitalWrite(aft[trig], LOW);

  digitalWrite(aft[echo], HIGH);
  aftDuration = pulseIn(aft[echo], HIGH);

  Serial.print("Fore: ");
  Serial.println(foreDuration);
  Serial.print("Aft: ");
  Serial.println(aftDuration);
  if (foreDuration > aftDuration) return 3; //needs to go right, can figure out what to return later
  if (aftDuration > foreDuration) return 2; //needs to go left, can figure out what to return later
}

//no real function, just displaying data
void loop()
{
  direct = wallCorrection();
  Serial.println(direct);
  delay(100);
}
