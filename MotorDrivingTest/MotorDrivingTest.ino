#define ENCA 2 
#define ENCB 3

#define PWM 10
#define In1 23
#define In2 22

#define BTN_PIN 7

int zeroTarget = 0;
int target = 12000; // 12100; // -12144

bool buttonState = LOW;
bool lastButtonState = LOW;

int setPosition = 0;
const int goToTop = 0;
const int goToBottom = 1;

int pos = 0; 

int dirDown = -1;
int dirUp = 1;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {
  
  buttonState = digitalRead(BTN_PIN);             // Active low
  //if (!button_state){
  //    pos = 0;
  //    delay(2000);
  //} 
  
  if (buttonState == HIGH && lastButtonState == LOW) {
    setPosition += 1;
    setPosition %= 2;
  }
  
  switch (setPosition) {
  case goToTop:
    if (pos > zeroTarget + 25){
    setMotor(dirDown, 40, PWM, In1, In2);
    }
    else if (pos < zeroTarget - 25){
      setMotor(dirUp, 40, PWM, In1, In2);
    }
    else{
      setMotor(0, 0, PWM, In1, In2);
    }
    break;
  case goToBottom:
    if (pos > target + 50){
      setMotor(dirDown, 70, PWM, In1, In2);
    }
    else if (pos < target - 50){
      setMotor(dirUp, 70, PWM, In1, In2);
    }
    else{
      setMotor(0, 0, PWM, In1, In2);
    }
    break;
  //default:
    // statements
  //  break;
  }

  lastButtonState = buttonState;

  //if (!button_state) setMotor(1, 30, PWM, In1, In2);    // Lift the egg
  //else setMotor(-1, 25, PWM, In1, In2);                 // Drop the egg
  
  Serial.println(pos);

  /*
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
  */


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
  if (dir == dirUp){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == dirDown){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else{
    digitalWrite(in1, LOW),
    digitalWrite(in2, LOW);
  }
}

// Write a function that detects the button press


