#define ENCA 2 
#define ENCB 3

#define PWM 10
#define In1 23
#define In2 22

int pos = 0; 

void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  //attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {

  for (int i = 0; i < 255; i++){
  digitalWrite(In1, 1);
  digitalWrite(In2, 0);
  analogWrite(PWM, i);
  delay(10);
  }
  for (int i = 255; i > 0; i--){
  digitalWrite(In1, 1);
  digitalWrite(In2, 0);
  analogWrite(PWM, i);
  delay(10);
  }
  
  //setMotor(1, 1, PWM, In1, In2);

  /*

  
  delay(1000);
  readLO();
  setMotor(-1, 25, PWM, In1, In2);
  delay(1000);
  readLO();
  setMotor(0, 0, PWM, In1, In2);
  delay(1000);
  readLO();
  */
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if (b>0){
    pos++;
  }
  else{
    pos--;
  }
}

void readLO(){
  Serial.print(digitalRead(39));
  Serial.println(digitalRead(37));
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm, pwmVal);
  if (dir == 1){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else{
    digitalWrite(in1, LOW),
    digitalWrite(in2, LOW);
  }
}
