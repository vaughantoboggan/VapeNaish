#include <Servo.h>
//global run time variables
bool goingForward = true, magFound = false;
int programStep = -1; //should start at 0
long nullTimer = 0;

//set global variables for the ultrasonic sensors
int fore[2] = {6, 7}; //first is trig, second is echo
int aft[2] = {12, 13}; //first is trig, second is echo
int trig = 0, echo = 1;
bool pingQueue = true;
long foreDuration = 0, aftDuration = 0, direct = 0, pingTimer = 0;

//set global variables for controlling motors
int rightSpeed = 5, rightDir1 = 10, rightDir2 = 11, leftSpeed = 3, leftDir1 = 10, leftDir2 = 11;

//set global variables for IR sensor
int irPin = 0, irVal = 0, Aval = 65, Eval = 69, Ival = 73, Oval = 79;
long irTimer = 0;

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
int left[2] = {9, 8}; //first is trig, second is echo
int right[2] = {2, 4}; //first is trig, second is echo
long leftDuration = 0, rightDuration = 0, frontTimer = 0;

//global variables for shimmy
long shimmyTimer = 0;
bool inShimmy = false, aligned = false, irSwitch = false;

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
  //setup the hall effect
  pinMode(A5, INPUT);
  // initialize serial communication:
  Serial.begin(2400);
}

//this function sets left motor speed and direction
void leftMotor(int velocity, bool polarity){ //false for reverse, true for forward
  if (polarity){
    digitalWrite(leftDir1, LOW);
    digitalWrite(leftDir2, HIGH);
  }
  else{
    digitalWrite(leftDir1, HIGH);
    digitalWrite(leftDir2, LOW);
  }
  analogWrite(leftSpeed, velocity);
}

//this function sets right motor speed and direction
void rightMotor(int velocity, bool polarity){ //false for reverse, true for forward
  if (polarity){
    digitalWrite(rightDir1, LOW);
    digitalWrite(rightDir2, HIGH);
  }
  else{
    digitalWrite(rightDir1, HIGH);
    digitalWrite(rightDir2, LOW);
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
  
  if (programStep < 5){
    if (leftDuration <= 1300 && rightDuration <= 1300 && leftDuration > 100 && rightDuration > 100 && leftDuration < (rightDuration + 20) && leftDuration >= (rightDuration-5)){
      return 1;
    }
    else return 2;
  }
  
  return 0;
}
//this function determines which direction to go when following the wall
int wallCorrection(double distance = 5){
  //int foreCorrect = 100;
  double analogDist = distance * 58; //converts centimeters to microsecond value
  long totalFore = 0, totalAft = 0;
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
  
  if (aftDuration > 0 && aftDuration < 10000 && foreDuration > 0 && foreDuration < 10000){
    if (programStep == 1){ //following the wall initially
      if (foreDuration > (aftDuration + 100)) return 3; //too much disparity, need to straighten out to the left
      if (aftDuration > (foreDuration + 100)) return 2; //too much disparity need to straighten out to the right
      if (foreDuration > (analogDist + 15) && aftDuration > (analogDist + 15)) return 3; //if too far away, need to go left to correct
      if (foreDuration < (analogDist - 15) && aftDuration < (analogDist - 15)) return 2; //if too close, need to go right to correct
      if (foreDuration > (aftDuration + 10)) return 3; //needs to go left, can figure out what to return later
      else if (foreDuration < (aftDuration - 10)) return 2; //needs to go right, can figure out what to return later
      else return 1; //just keep on trucking  
    }
    if (programStep == 2){ //correcting after finding the cube
      if (aftDuration > foreDuration && aftDuration > 350){
        return 1;
      }
      else if (foreDuration > aftDuration && foreDuration > 250){
        return 2;
      }
      else return 5;
    }
    if (programStep == 3){ //pulling up alongside the cube
      if (foreDuration > 270) return 1;
      else if (aftDuration > 370) return 2;
      else return 3;
    }
  }
  else return 0;
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

//this function cleans out the corner at the onset of the program
void cleanTheCorner(){
  //if (millis() < (cleanTimer + 5000)){} this is the section for sweeping as far as we can
  if (millis() < (cleanTimer + 600)){ //pull away from the wall going back right, previous + 800
    leftMotor(110, goingForward);//need to be switched?
    rightMotor(110, goingForward);
  }
  else if (millis() < (cleanTimer + 1100)){
    leftMotor(0, goingForward);
    rightMotor(0, goingForward);
    cleanSweep();
  }
  else if (millis() < (cleanTimer + 1600)){
    leftMotor(0, goingForward);
    rightMotor(0, goingForward);
    cleanSweep();
  }
  else if (millis() < (cleanTimer + 2400)){
    leftMotor(110, !goingForward);
    rightMotor(0, !goingForward);
    cleanSweep();
  }
  else if (millis() < (cleanTimer + 6400)){
    leftMotor(110, !goingForward);
    rightMotor(110, !goingForward);
    cleanSweep();
  }
  else{ //stop for debugging purposes, continue on
    leftMotor(0, goingForward);
    rightMotor(0, goingForward);
    cleanSweep();
    programStep++;
  }
}

//this function opens the bottom hatch slowly
void openHatch(){
  int i = 180;
  while (i >= 60){
    hatchServo.write(i);
    if (millis() %75 == 0){
      i--;
    }
  }
}

//this function closes the bottom hatch
void closeHatch(){
  hatchServo.write(180);
}

//raise the arm vertically
void verticalArm(){
  armServo.write(5);
}
//give the arm a slight angle so it can see the wall
void seekingArm(){
  armServo.write(20);
}
//hold the arm at an angle so ultrasonics can sense pyramid
void angleArm(){
  armServo.write(40);
}

//lay out the arm horizontally
void horizontalArm(){
  armServo.write(92);
}

//sweep the cube off the wall with the sweep servo
void cleanSweep(){
  sweepServo.write(0);
}
//return sweep arm to trailing position
void trailingSweep(){
  sweepServo.write(105);
}
//set sweep at a neutral position
void neutralSweep(){
  sweepServo.write(60);
}
//close the claw
void closeClaw(){
  clawServo.write(180);
}
//open the claw
void openClaw(){
  clawServo.write(60);
}
//hall effect sensing code as a boolean
bool checkMag(){
  int magData = analogRead(A5); //get the data 
  if (510 > magData || magData > 521) return true; //if cube is present
  else return false; //if cube is not
}
//correct trajectory when following IR
void irCorrection(){
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
  
  if (millis() >= (frontTimer + 30) && !frontQueue){
    digitalWrite(right[trig], LOW);
    delayMicroseconds(2);
    digitalWrite(right[trig], HIGH);
    delayMicroseconds(5);
    digitalWrite(right[trig], LOW);

    digitalWrite(right[echo], HIGH);
    rightDuration = pulseIn(right[echo], HIGH);
    frontQueue = true;
  }
  //do the shimmy corrections
  if (programStep == 7){
    if (leftDuration < 1000 && rightDuration > 1000 && leftDuration > 100 && !aligned){
      inShimmy = true;
      shimmyTimer = millis();
      while(inShimmy){
        shimmyLeft();
      }
    }
    if (rightDuration < 1000 && leftDuration > 1000 && rightDuration > 100 && !aligned){
      inShimmy = true;
      shimmyTimer = millis();
      while(inShimmy){
        shimmyRight();
      }
    }
    if (rightDuration < 1000 && leftDuration < 1000 && leftDuration > 100 && rightDuration > 100 && !aligned){
      leftMotor(0, goingForward);
      rightMotor(0, goingForward);
      aligned = true;
    }
    if (aligned){
      leftMotor(0, goingForward);
      rightMotor(0, goingForward);
    }
  }
  if (programStep == 8){
    if (leftDuration < (rightDuration - 50)){
      rightMotor(150, goingForward);
      leftMotor(0, goingForward);
    }
    else if (leftDuration > (rightDuration + 50)){
      leftMotor(150, goingForward);
      rightMotor(0, goingForward);
    }
    else if (leftDuration < 1000 && rightDuration < 1000){
      leftMotor(0, goingForward);
      rightMotor(0, goingForward);
      programStep+=2;
      nullTimer = millis();
    }
    else{
      leftMotor(0, goingForward);
      rightMotor(0, goingForward);
      programStep++;
      nullTimer = millis();
      Serial.print("Left: ");
      Serial.println(leftDuration);  
      Serial.print("Right: ");
      Serial.println(rightDuration);
    }
  }
}

void shimmyLeft(){
  inShimmy = true;
  if (millis() < (shimmyTimer + 200)){
    rightMotor(150, !goingForward);
    leftMotor(0, !goingForward);
  }
  else if (millis() < (shimmyTimer + 700)){
    rightMotor(150, !goingForward);
    leftMotor(150, !goingForward);
  }
  else if (millis() < (shimmyTimer + 1150)){
    rightMotor(150, goingForward);
    leftMotor(0, goingForward);
  }
  else if (millis() < (shimmyTimer + 1550)){
    rightMotor(150, goingForward);
    leftMotor(150, goingForward);
  }
  else{
    rightMotor(0, goingForward);
    leftMotor(0, goingForward);
    inShimmy = false;
  }
}

void shimmyRight(){
  inShimmy = true;
  if (millis() < (shimmyTimer + 200)){
    rightMotor(0, !goingForward);
    leftMotor(150, !goingForward);
  }
  else if (millis() < (shimmyTimer + 700)){
    rightMotor(150, !goingForward);
    leftMotor(150, !goingForward);
  }
  else if (millis() < (shimmyTimer + 900)){
    rightMotor(0, goingForward);
    leftMotor(150, goingForward);
  }
  else if (millis() < (shimmyTimer + 1400)){
    rightMotor(150, goingForward);
    leftMotor(150, goingForward);
  }
  else{
    rightMotor(0, goingForward);
    leftMotor(0, goingForward);
    inShimmy = false;
  }
}
//IR sensing data
bool retrieveIR(){
  int temp = 0;
  if (Serial.available()){
    temp = Serial.read();
  }
  else{
    temp = 0;
  }
  if (temp!=0){
    Serial.println(temp);
    if (irSwitch){
      if (temp == Aval || temp == Eval || temp > 100){
        return true;
      }
    }
    else{
      if (temp == Ival || temp == Oval || temp > 100){
        return true;
      }
    }
  }
  else return false;
}
bool getBearing(){
  if (irTimer == 0) irTimer = millis();
  if (millis() >= (irTimer + 250)){
    bool check = retrieveIR();
    if (check){
      return true;
    }
    else{
      return false;
    }
    irTimer = millis();
  }
}






//run time function
void loop()
{
  if (millis() > 5000 && programStep == -1){
    programStep = 1;
  }
  if (programStep == 0) verticalArm();
  else if (programStep <= 5) seekingArm();
  else if (programStep < 10) angleArm();
  //if (programStep >= 6) cleanSweep();
  if (programStep < 10) openClaw();
  if (programStep < 10) closeHatch();
  if (!magFound && programStep != 0 && programStep < 6) neutralSweep();
  if (programStep < 5){
    int frontCheck = frontPings();
    if (frontCheck == 1 && !magFound){
      leftMotor(0, goingForward);
      rightMotor(0, goingForward);
      programStep = 5;
      nullTimer = millis();
      if (cleanTimer == 0) cleanTimer = millis();
    }
  }
  if (programStep == 0){
    if (nullTimer == 0) nullTimer = millis();
    if (millis() < (nullTimer + 1200)){
      trailingSweep();
      leftMotor(110, goingForward);
      rightMotor(110, goingForward);
    }
    else if (millis() < (nullTimer + 1700)){
      trailingSweep();
      leftMotor(110, !goingForward);
      rightMotor(80, !goingForward);
    }
    else if (millis() < (nullTimer + 2400)){
      leftMotor(0, goingForward);
      rightMotor(0, goingForward);
      cleanSweep();
    }
    else if (millis() < (nullTimer + 2800)){
      leftMotor(110, !goingForward);
      rightMotor(110, !goingForward);
    }
    else{
        leftMotor(0, goingForward);
        rightMotor(0, goingForward);
        programStep++;
        nullTimer = millis();
    }
  }
  if (programStep == 1){
    direct = wallCorrection(4); //calls on correction function, parameter is distance in cm
    //Serial.println(direct);
    if (checkMag()){
      leftMotor(0, goingForward);
      rightMotor(0, goingForward);
      direct = 0;
      magFound = true;
      programStep++;
      nullTimer = millis();
    }
    if (direct == 3){ //needs to turn left
      leftMotor(0, goingForward);
      rightMotor(110, goingForward);
    }
    if (direct == 2){ //needs to turn right
      leftMotor(110, goingForward);
      rightMotor(0, goingForward);
    }
    if (direct == 1){ //needs to go straight on
      leftMotor(110, goingForward);
      rightMotor(110, goingForward);
    }
  }
  if (programStep == 2){
    direct = wallCorrection();
    if (direct == 1){
      leftMotor(0, !goingForward);
      rightMotor(110, !goingForward);
    }
    if (direct == 2){
      leftMotor(110, !goingForward);
      rightMotor(0, !goingForward);
    }
    if (direct == 5){
      leftMotor(0, goingForward);
      rightMotor(0, goingForward);
      programStep++;
      nullTimer = millis();
    }
  }
  if (programStep == 3){
    if (millis() < (nullTimer + 800)){
      leftMotor(100, goingForward);
      rightMotor(110, goingForward);
    }
    else{
      leftMotor(0, goingForward);
      rightMotor(0, goingForward);
      programStep++;
      nullTimer = millis();
    }
  }
  if (programStep == 4){
    leftMotor(0, goingForward);
    rightMotor(0, goingForward);
    if (millis() < (nullTimer + 500)){
      sweepServo.write(170);
    }
    else if (millis() < (nullTimer + 1000)){
      cleanSweep();
    }
    else if (millis() < (nullTimer + 2800)){
      leftMotor(110, !goingForward);
      rightMotor(0, !goingForward);
    }
    else if (millis() < (nullTimer + 6600)){
      leftMotor(110, !goingForward);
      rightMotor(110, !goingForward);
    }
    else{
      leftMotor(0, goingForward);
      rightMotor(0, goingForward);
      programStep+=2;
    }
  }
  if (programStep == 5){
    cleanTheCorner();
  }
  if (programStep == 6){
    rightMotor(150, goingForward);
    leftMotor(0, goingForward);
    bool temp = getBearing();
    if (temp) {
      rightMotor(0, goingForward);
      leftMotor(0, goingForward); 
      programStep++;
    }
  }
  if (programStep == 7){
    rightMotor(150, goingForward);
    leftMotor(150, goingForward);    
    if (millis() > 600 && !aligned) irCorrection();
    if (aligned){
      rightMotor(0, goingForward);
      leftMotor(0, goingForward);
      programStep++;
    }
  }
  if (programStep == 8){
    irCorrection(); 
  }
  if (programStep == 9){
    if (millis() < (nullTimer + 450)){
      rightMotor(150, !goingForward);
      leftMotor(150, !goingForward);
    }
    else{
      nullTimer = millis();
      programStep++;
    }
  }
  if (programStep == 10){
    rightMotor(0, !goingForward);
    leftMotor(0, !goingForward);
    if (millis() < (nullTimer + 2000)){
      horizontalArm();
    }
    else angleArm();
    if (millis() > (nullTimer + 1000)){
      closeClaw();
    }
    //if (millis() < (nullTimer + 5000)){
    //  closeHatch();
    //}
    if (millis() >= (nullTimer + 5000)){
      openHatch();
      closeClaw();
      nullTimer = millis();
      programStep++;
    }
  }
  if (programStep == 11){
    angleArm();
    closeClaw();
    if (millis() < (nullTimer + 300)){
      rightMotor(0, !goingForward);
      leftMotor(0, !goingForward);
    }
    else if (millis() < (nullTimer + 900)){
      rightMotor(110, !goingForward);
      leftMotor(0, !goingForward);
    }
    else if (millis() < (nullTimer + 2000)){
      rightMotor(110, !goingForward);
      leftMotor(110, !goingForward);
    }
    else{
      leftMotor(0, !goingForward);
      rightMotor(0, !goingForward);
      nullTimer = millis();
      programStep++;
    }
  }
  if (programStep == 12){
    if (millis() < (nullTimer + 1500)){
      leftMotor(0, !goingForward);
      rightMotor(0, !goingForward);
      horizontalArm();
    }
    else if (millis() < (nullTimer + 3000)){
      leftMotor(0, !goingForward);
      rightMotor(0, !goingForward);
      openClaw();
    }
    else{
      leftMotor(0, !goingForward);
      rightMotor(0, !goingForward);
      verticalArm();
    }
  }
  //if (programStep == 13){
  //  cleanSweep();
  //}
}
