const int IRsensor = 0;
int IRread = 0;

int A = 65;
int E = 69;
int I = 73;
int O = 79;
void setup() {
  pinMode(IRsensor, INPUT);
  Serial.begin(2400);
}

void loop() {
  Serial.print(" IR Sensor: ");
  Serial.available();
  IRread = Serial.read();
  if(IRread == A)
    Serial.println("A");
  if(IRread == E)
    Serial.println("E");
  if(IRread == I)
    Serial.println("I");
  if(IRread == O)
    Serial.println("O");
}
