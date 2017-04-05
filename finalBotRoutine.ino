#include <Servo.h>
//global run time variables
bool goingForward = true, magFound = false;
int programStep = 0;
long nullTimer = 0;

//set global variables for the ultrasonic sensors
int fore[2] = {6, 7}; //first is trig, second is echo
int aft[2] = {12, 13}; //first is trig, second is echo
int trig = 0, echo = 1;
bool pingQueue = true;
long foreDuration = 0, aftDuration = 0, direct = 0, pingTimer = 0;

//set global variables for controlling motors
int rightSpeed = 9, rightDir1 = 10, rightDir2 = 11, leftSpeed = 8, leftDir1 = 10, leftDir2 = 11;

//set global variables for IR sensor
int irPin = 0, irVal = 0, Aval = 65, Eval = 69, Ival = 73, Oval = 79;

//set global variables for bearingSpiral
long spiralTimer = 0;
bool departing = true, tooClose = false;
int forwardMod = 0;

//set global variables for cleaning the corner
long cleanTimer = 0, cleanProgression = 0;

//set global variables for the hatch servo
Servo hatchServo;

//set global variables for the sweep servo
Servo sweepServo;

//set global variables for the arm servo
Servo armServo;

//set global variables for the claw servo
Servo clawServo;

//set global variables for the front ultrasonics
bool frontQueue = true;
int left[2] = {2, 3}; //first is trig, second is echo
int right[2] = {4, 5}; //first is trig, second is echo
long leftDuration = 0, rightDuration = 0, frontTimer = 0;
void setup() {
  //set pin modes for the fore ultrasonic
  pinMode(fore[0], OUTPUT);
  pinMode(fore[1], INPUT);
  //set pin modes for the aft ultrasonic
  pinMode(aft[0], OUTPUT);
  pinMode(aft[1], INPUT);
  //set pin modes for the left ultrasonic
  pinMode(left[0], OUTPUT);
  pinMode(left[1], INPUT);
  //set pin modes for the right ultrasonic
  pinMode(right[0], OUTPUT);
  pinMode(right[1], INPUT);
  //set pin modes for the left motor
  pinMode(leftSpeed, OUTPUT);
  pinMode(leftDir1, OUTPUT);
  pinMode(leftDir2, OUTPUT);
  //set pin modes for the right motor
  pinMode(rightSpeed, OUTPUT);
  pinMode(rightDir1, OUTPUT);
  pinMode(rightDir2, OUTPUT);
  //set pin mode for the IR sensor
  pinMode(irPin, INPUT);
  //setup the hatch servo
  hatchServo.attach(A1);
  //setup the sweep servo
  sweepServo.attach(A2);
  //setup the arm servo
  armServo.attach(A0);
  //setup the claw servo
  clawServo.attach(A3);
  // initialize serial communication:
  Serial.begin(2400);
}

//this function sets left motor speed and direction
void leftMotor(int velocity, bool polarity){ //false for reverse, true for forward
  if (polarity){
    digitalWrite(leftDir1, HIGH);
    digitalWrite(leftDir2, LOW);
  }
  else{
    digitalWrite(leftDir1, LOW);
    digitalWrite(leftDir2, HIGH);
  }
  analogWrite(leftSpeed, velocity);
}

//this function sets right motor speed and direction
void rightMotor(int velocity, bool polarity){ //false for reverse, true for forward
  if (polarity){
    digitalWrite(rightDir1, HIGH);
    digitalWrite(rightDir2, LOW);
  }
  else{
    digitalWrite(rightDir1, LOW);
    digitalWrite(rightDir2, HIGH);
  }
  analogWrite(rightSpeed, velocity);
}

//this function determines the distance for left and right front ultrasonics
int frontPings(){
  if (frontQueue){
    frontTimer = millis();
    digitalWrite(left[trig], LOW);
    delayMicroseconds(2);
    digitalWrite(left[trig], HIGH);
    delayMicroseconds(5);
    digitalWrite(left[trig], LOW);

    digitalWrite(left[echo], HIGH);
    leftDuration = pulseIn(left[echo], HIGH);
    frontQueue = false;
  }
  
  if (millis() >= (frontTimer + 50) && !frontQueue){
    digitalWrite(right[trig], LOW);
    delayMicroseconds(2);
    digitalWrite(right[trig], HIGH);
    delayMicroseconds(5);
    digitalWrite(right[trig], LOW);

    digitalWrite(right[echo], HIGH);
    rightDuration = pulseIn(right[echo], HIGH);
    frontQueue = true;
  }
  Serial.print("Left: ");
  Serial.println(leftDuration);
  Serial.print("Right: ");
  Serial.println(rightDuration);

  return 1;
}
//this function determines which direction to go when following the wall
int wallCorrection(double distance = 5){
  int foreCorrect = 100;
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
  //foreDuration+=foreCorrect;  
  Serial.print("Fore: ");
  Serial.println(foreDuration);
  Serial.print("Aft: ");
  Serial.println(aftDuration);
  if (foreDuration > (aftDuration + 100)) return 3; //too much disparity, need to straighten out to the left
  if (aftDuration > (foreDuration + 100)) return 2; //too much disparity need to straighten out to the right
  if (foreDuration > (analogDist + 15) && aftDuration > (analogDist + 15)) return 3; //if too far away, need to go left to correct
  if (foreDuration < (analogDist - 15) && aftDuration < (analogDist - 15)) return 2; //if too close, need to go right to correct
  if (foreDuration > (aftDuration + 10)) return 3; //needs to go left, can figure out what to return later
  else if (foreDuration < (aftDuration - 10)) return 2; //needs to go right, can figure out what to return later
  else return 1; //just keep on trucking
}

//this function spirals the bot out from the wall in order to get a bearing
void bearingSpiral(){
  if (departing) forwardMod = 3500;
  else forwardMod = -3300;
  if (tooClose){
    //this is where we put the code to back up in case it's too close to a wall in front of it
  }
  else{
    if(millis() < (spiralTimer + 1800) && departing){
      leftMotor(200, !goingForward);
      rightMotor(0, !goingForward);
    }
    else if(millis() < (spiralTimer + forwardMod) && departing){
      leftMotor(200, !goingForward);
      rightMotor(200, !goingForward);
    }
    else if(millis() < ((long)(spiralTimer + forwardMod + 11500))){
      leftMotor(200, goingForward);
      rightMotor(0, goingForward);
    }
    else if (millis() < ((long)(spiralTimer + forwardMod + 13000))){
      leftMotor(200, goingForward);
      rightMotor(200, goingForward);
    }
    else{
      leftMotor(0, goingForward);
      rightMotor(0, goingForward);
      departing = false;
      spiralTimer = millis();
    }
  }
}

//while spiralling, this function detects if the bot picks up an IR signal
int getBearing(){
  int temp = 0;
  if (Serial.available()){
    temp = Serial.read();
  }
  else{
    temp = 0;
  }
  return temp;
}

//this function cleans out the corner at the onset of the program
void cleanTheCorner(){
  //if (millis() < (cleanTimer + 5000)){} this is the section for sweeping as far as we can
  if (millis() < (cleanTimer + 800)){ //pull away from the wall going back right, previous + 800
    leftMotor(200, !goingForward);//need to be switched?
    rightMotor(0, !goingForward);
  }
  else if (millis() < (cleanTimer + 2075)){ //go backwards straight to give distance, previous + 1275
    leftMotor(200, !goingForward);
    rightMotor(200, !goingForward);
  }
  else if (millis() < (cleanTimer + 2825)){ //straighten out, previous + 750
    leftMotor(200, goingForward);
    rightMotor(0, goingForward);
  }
  else if (millis() < (cleanTimer + 4525)){ //pull up alongside the wall, previous + 1700
    leftMotor(200, goingForward);
    rightMotor(0, goingForward);
  }
  //else if (millis() < (cleanTimer + 35000)){ //this is the section for pulling the sweep arm back as far as we can
  //}
  else if (millis() < (cleanTimer + 4825)){ //back up to the corner, previous + 300
    leftMotor(200, !goingForward);
    rightMotor(200, !goingForward);
  }
  //else if (millis() < (cleanTimer + 40000)){} sweep into the bucket
  else if (millis() < (cleanTimer + 6325)){ //go forward to clear the back, previous + 1500
    leftMotor(200, goingForward);
    rightMotor(200, goingForward);
  }
  else{ //stop for debugging purposes
    leftMotor(0, goingForward);
    rightMotor(0, goingForward);
  }
}

//this function opens the bottom hatch slowly
void openHatch(){
  int i = 80;
  while (i >= 0){
    hatchServo.write(i);
    if (millis() %75 == 0){
      i--;
    }
  }
}

//this function closes the bottom hatch
void closeHatch(){
  hatchServo.write(80);
}

//raise the arm vertically
void verticalArm(){
  armServo.write(25);
}

//hold the arm at an angle so ultrasonics can sense pyramid
void angleArm(){
  armServo.write(42);
}

//lay out the arm horizontally
void horizontalArm(){
  armServo.write(93);
}

//sweep the cube off the wall with the sweep servo
void cleanSweep(){
  sweepServo.write(0);
}
//return sweep arm to resting position
void returnSweep(){
  sweepServo.write(125);
}
//run time function
void loop()
{
  armServo.write(5);
  //this is the program step for following along the wall
  if (programStep == 0){
    direct = wallCorrection(6.95); //calls on correction function, parameter is distance in cm
    Serial.println(direct);
    if (millis() %1000 == 0 && millis() %3000!=0) cleanSweep();
    else if(millis()%3000 == 0) returnSweep();
    //if (direct == 3){ //needs to turn left
    //  leftMotor(115, goingForward);
    //  rightMotor(150, goingForward);
    //}
    //if (direct == 2){ //needs to turn right
    //  leftMotor(150, goingForward);
    //  rightMotor(115, goingForward);
    //}
    //if (direct == 1){ //needs to go straight on
    //  leftMotor(140, goingForward);
    //  rightMotor(140, goingForward);
    //}
  }
  //this is the step for sweeping the pyramid off the final corner, will not be a program step in practice
  if (programStep == 1){
    if (cleanTimer == 0) cleanTimer = millis();
    cleanTheCorner();
  }
  //this is the program step for finding the pyramid and launching away from the wall
  if (programStep == 2){
    if (spiralTimer == 0) spiralTimer = millis();
    bearingSpiral();
    if (nullTimer == 0) nullTimer = millis();
    if (millis() >= (nullTimer + 250)){
      irVal = getBearing();
      nullTimer = millis();
    }
    if (true){ //comparator for whether we  are using A/E or I/O IR Beacon
      if (irVal == Aval || irVal == Eval){
        programStep++;
      }
    }
    else{
      if (irVal == Ival || irVal == Oval){
        programStep++;
      }
    }
  }
  if (programStep == 3){
    //frontPings();
    //if (millis() %1000 == 0 && millis() %3000!=0) cleanSweep();
    //else if(millis()%3000 == 0) returnSweep();
  }
}
