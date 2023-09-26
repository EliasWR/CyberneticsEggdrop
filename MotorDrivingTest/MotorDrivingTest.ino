#include <Arduino.h>

const int LOG = 1;
const int DEBUG = 2;
const int OFF = 0;
int LOGGING = LOG;

/*==== Pinout ====*/
#define ENCA 2 
#define ENCB 3

#define PWM 10
#define In1 23
#define In2 22

#define BTN_PIN 7
/*=================*/


/*==== Target ====*/ 
const int nTARGETS = 5;
int targetNum = 0;
int targetList[] = {5000, 7000, 9000, 10000, 11000};
int target = 11000; // 12100; // -12144
/*================*/

// ==== Timer ====
unsigned long nextTimeout = 0;
unsigned long BFnextTimeout = 0;
unsigned long stateTimer = 0;
// ================


const int ENCODER_RESOLUTION = 2048;
const int SPEED = 50;
const int SPEED_TO_ZERO = 30;
int motorSpeed = 0;
unsigned long timestamp = 0;
int target_log = 0;
const int zeroTarget = 0;
const int zeroThreshold = 150;
const int targetThreshold = 150;

// ==== Button ====
// Active low
bool buttonState = HIGH;
bool lastButtonState = HIGH;


/* ==== Position Control ==== */
int setPosition = 0;
const int goToTop = 0;
const int goToBottom = 1;
int current_pos = 0; 
const int DOWN = -1;
const int UP = 1;
const int sSTOP = 0;
const int sTO_ZERO = 1;
const int sTO_TARGET = 2;
int current_state = sSTOP;
/* ========================== */

void setup() {
  digitalWrite(13, LOW);
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  setMotor(0, 0, PWM, In1, In2);
  /*___ Set Zero at startup___*/
  // Position the egg at desired ZERO. Then press button to move on
  if(LOGGING == DEBUG) Serial.println("Press button to set zero");
  buttonState = digitalRead(BTN_PIN);
  while (!(buttonState == HIGH && lastButtonState == LOW)){
    lastButtonState = buttonState;
    buttonState = digitalRead(BTN_PIN);
  }
  if (LOGGING == DEBUG) Serial.println("Zero set");
  lastButtonState = buttonState;
  current_pos = 0;
  current_state = sSTOP;
  delay(2000);
}

void loop() {
  
  /*___ Handle button press ___*/
  buttonState = digitalRead(BTN_PIN);             // Active low

  /*
  if (buttonState == HIGH && lastButtonState == LOW && buttonFilterHasExpired()) {
    startButtonFilter(100);
    
    setPosition += 1;
    setPosition %= 2;
    targetNum += 1;
    targetNum %= 5;
    target = targetList[targetNum];
    
  }
  /*


  /*___ STATE MACHINE ___*/
  switch(current_state){
    case sSTOP:
      if (LOGGING == DEBUG) Serial.println("STOP");
      setMotor(0, 0, PWM, In1, In2);
      if (buttonState == HIGH && lastButtonState == LOW && buttonFilterHasExpired()) {
        // On button RELEASE
        startButtonFilter(100);
        current_state = sTO_ZERO;
      }
      break;

    case sTO_ZERO:
      if (LOGGING == DEBUG) Serial.println("TO ZERO");
      target_log = zeroTarget;
      if (goToZero() && buttonState == HIGH && lastButtonState == LOW && buttonFilterHasExpired()){
        startButtonFilter(100);
        current_state = sTO_TARGET;
        target = targetList[targetNum++];
        
        targetNum %= nTARGETS;

        if (targetNum == 0) {current_state = sSTOP;}
        else {current_state = sTO_TARGET;}
      }
      break;

    case sTO_TARGET:
      if (LOGGING == DEBUG) Serial.println("TO TARGET");
      target_log = target;
      if (goToTarget(target, targetThreshold, SPEED) && buttonState == HIGH && lastButtonState == LOW && buttonFilterHasExpired()){
        startButtonFilter(100);
        
        current_state = sTO_ZERO;
      }
      break;

  }

  /*___ Print to serial ___*/
  if (LOGGING == LOG){
    if (timerHasExpired()){
      startTimer(15);
      Serial.print(millis());
      Serial.print(", ");
      Serial.print(target_log);
      Serial.print(", ");
      Serial.print(motorSpeed);
      Serial.print(", ");
      Serial.println(current_pos);
    }
  }

  /*___ STATE MACHINE ___*/
  /*
  switch (setPosition) {
  case goToTop:
    if (current_pos > zeroTarget + 200){
    setMotor(DOWN, 30, PWM, In1, In2);
    }
    else if (current_pos < zeroTarget - 200){
      setMotor(UP, 30, PWM, In1, In2);
    }
    else{
      setMotor(0, 0, PWM, In1, In2);
    }
    target_log = zeroTarget;
    break;
  case goToBottom:
    if (current_pos > target + 150){
      setMotor(DOWN, 60, PWM, In1, In2);
    }
    else if (current_pos < target - 150){
      setMotor(UP, 60, PWM, In1, In2);
    }
    else{
      setMotor(0, 0, PWM, In1, In2);
    }
    target_log = target;
    break;
  //default:
    // statements
  //  break;
  }
  */
 

  

  //if (!button_state) setMotor(1, 30, PWM, In1, In2);    // Lift the egg
  //else setMotor(-1, 25, PWM, In1, In2);                 // Drop the egg
  

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

  lastButtonState = buttonState;
}


void startTimer(int duration) {
      nextTimeout = millis() + duration;
    }


bool timerHasExpired() {
  bool timerExpired = (millis() >= nextTimeout);
  return timerExpired;
}


void startButtonFilter(int duration) {
      BFnextTimeout = millis() + duration;
    }


bool buttonFilterHasExpired() {
  bool timerExpired = (millis() >= BFnextTimeout);
  return timerExpired;
}


void readEncoder(){
  int b = digitalRead(ENCB);
  if (b>0){
    current_pos++;
  }
  else{
    current_pos--;
  }
}


bool goToTarget(int target, int threshold, int speed){
  if (current_pos > (target + threshold)){
    setMotor(DOWN, speed, PWM, In1, In2);
    }
  else if (current_pos < (target - threshold)){
    setMotor(UP, speed, PWM, In1, In2);
  }
  else{
    //setMotor(0, 0, PWM, In1, In2);
    holdMotor();
    return true;
  }
  return false;
}


bool goToZero(){
  return goToTarget(zeroTarget, zeroThreshold, SPEED_TO_ZERO);
}


void holdMotor(){
  setMotor(UP, 0, PWM, In1, In2);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  motorSpeed = pwmVal;
  analogWrite(pwm, pwmVal);
  if (dir == UP){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == DOWN){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else{
    digitalWrite(in1, LOW),
    digitalWrite(in2, LOW);
  }
}
