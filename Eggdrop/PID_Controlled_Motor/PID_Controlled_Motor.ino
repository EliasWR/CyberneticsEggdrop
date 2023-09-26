//#include <AVR_RTC.h>



//*************************************
// DC Motor PID position control example
// By Øystein Bjelland, IIR, NTNU
// Based on this example: https://curiores.com/dc-motor-control/ 
//**************************************

#include <util/atomic.h>
#include "pid.h"
#include "motor.h"
#include "timer.h"

//**************************************

#define ENCA 2        //Encoder pinA
#define ENCB 3        //Encoder pinB
#define PWM 10        //motor PWM pin
#define IN2 23        //motor controller pin2
#define IN1 22        //motor controller pin1
#define BTN_PIN 7     //button pin


volatile int32_t posi = 0; // position variable. https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/ 

// System identification
// Kp = 0.0991;
// Ki = 0.0126;
float Kp = 0.12; //Proportional gain // 0.12 // 0.18 
float Ki = 0.06; //Integral gain // 0.06 // 0.01 // 0.12
float Kd = 0.0; //Derivative gain
float motorSpeedMax = 255;

bool last_btn_state = HIGH; //Button state
int32_t current_pos = 0; //Current position


/* STATE MACHINE */

int last_state = 0;
enum states {IDLE, SET_TARGET, SET_START_POS, RUN, RUN_TO_START, READY_FOR_DROP};
int state = IDLE;


/*****************   OBJECTS   ******************/
PID pid(Kp, Ki, Kd, -motorSpeedMax, motorSpeedMax);  //PID controller
Motor motor(PWM, IN1, IN2);  //DC motor
Timer StateTimer; //Timer
/************************************************/

int target = 0;
int target_threshold = 100;
int start_pos = 0;
int dir = 1;
float u = 0;

void setup() {
  
  Serial.begin (115200);

  // ENCODER
  pinMode (ENCA, INPUT);
  pinMode (ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING); //https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/

  // DC MOTOR
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // BUTTON
  pinMode(BTN_PIN, INPUT_PULLUP);

  pid.reset();

  setState(SET_TARGET);
  
  
}

void loop(){
  bool btn_state = digitalRead(BTN_PIN);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    current_pos = posi;
  } 

  switch (state)
  {
  case IDLE:
    motor.stop();
    if(btn_state && !last_btn_state){
      setState(SET_START_POS);
    }
    break;

  case SET_TARGET:
    motor.free();
    if(btn_state && !last_btn_state){
      target = current_pos;
      setState(SET_START_POS);
    }
    break;
  
  case SET_START_POS:
    motor.free();
    if(btn_state && !last_btn_state){
      start_pos = current_pos;
      motor.stop();
      setState(READY_FOR_DROP);
    }
    break;

  case READY_FOR_DROP:
    motor.free();
    if(btn_state && !last_btn_state){
      setState(RUN);
    }
    break;

  case RUN_TO_START:
    u = pid.update(current_pos, start_pos);
    motor.run(u);
    /*
    Serial.print("Target pos: ");
    Serial.print(target);
    Serial.print(", Encoder pos: ");
    Serial.print(current_pos);
    Serial.print(", PID out: ");
    Serial.println(u);
    */
    Serial.print(millis());
    Serial.print(", ");
    Serial.print(target);
    Serial.print(", ");
    Serial.print(u);
    Serial.print(", ");
    Serial.println(current_pos);
    if((current_pos <= start_pos + target_threshold && current_pos >= start_pos - target_threshold) or (btn_state && !last_btn_state)){
      motor.stop();
      setState(READY_FOR_DROP);
    }
    break;

  case RUN:
    u = pid.update(current_pos, target);
    motor.run(u);
    /*
    Serial.print("Target pos: ");
    Serial.print(target);
    Serial.print(", Encoder pos: ");
    Serial.print(current_pos);
    Serial.print(", PID out: ");
    Serial.println(u);
    */
    Serial.print(millis());
    Serial.print(", ");
    Serial.print(target);
    Serial.print(", ");
    Serial.print(u);
    Serial.print(", ");
    Serial.println(current_pos);
    if(btn_state && !last_btn_state){
      setState(RUN_TO_START);
    }
    break;

  default:
    break;
  }

  last_btn_state = btn_state;
}


/*
void loop() {
  bool btn_state = digitalRead(BTN_PIN);
  while(!btn_state){
    btn_state = digitalRead(BTN_PIN);
    motor.free();

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      current_pos = posi;
    } 

    Serial.print("Encoder pos: ");
    Serial.println(current_pos);
  }

  
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    current_pos = posi;
  } 
  auto u = pid.update(current_pos, target);

  motor.run(u);

  Serial.print("Target pos: ");
  Serial.print(target);
  Serial.print(", Encoder pos: ");
  Serial.print(current_pos);
  Serial.print(", PID out: ");
  Serial.print(u);
  Serial.println("");

  /*

  // Print for logging
  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println(" ");
//  Serial.print(e);
//  Serial.print(" ");
//  Serial.print(u);
//  Serial.print(" ");
//  Serial.print(dir);
//  Serial.print(" ");
//  Serial.println(pwr);
  






  last_btn_state = btn_state;
}

//******************************************
//FUNCTIONS FOR MOTOR AND ENCODER

*/

//ENCODER
void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}


void setState(int new_state){
  last_state = state;
  state = new_state;
  /*
  Serial.print("State changed from ");
  Serial.print(stateStr(last_state));
  Serial.print(" to ");
  Serial.println(stateStr(state));
  Serial.print("DeltaT: ");
  Serial.println(StateTimer.get_deltaT());
  Serial.print("Current position: ");
  Serial.println(current_pos);
  Serial.println("");
  */
  StateTimer.reset();
}

void setState(int new_state, int delay){
  last_state = state;
  state = new_state;
  /*
  Serial.print("State changed from ");
  Serial.print(stateStr(last_state));
  Serial.print(" to ");
  Serial.println(stateStr(state));
  Serial.print("DeltaT: ");
  Serial.println(StateTimer.get_deltaT());
  Serial.print("Current position: ");
  Serial.println(current_pos);
  Serial.println("");
  */
  StateTimer.reset(delay);
}


//STATES
String stateStr(int state){
  switch (state)
  {
  case IDLE:
    return "IDLE";
    break;
  case SET_TARGET:
    return "SET_TARGET";
    break;
  case SET_START_POS:
    return "SET_START_POS";
    break;
  case RUN:
    return "RUN";
    break;
  case RUN_TO_START:
    return "RUN_TO_START";
    break;
  case READY_FOR_DROP:
    return "READY_FOR_DROP";
    break;
  default:
    return "UNKNOWN";
    break;
  }
}
