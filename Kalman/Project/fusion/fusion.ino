// PC: 192.168.10.20
// Arduino: 192.168.10.240
// Subnet: 255.255.255.0

#include <HardWire.h>
#include <I2C_MPU6886.h>
#include <VL53L0X.h>

#include <Ethernet.h>
#include <EthernetUdp.h>

#include <util/atomic.h>
#include "pid.h"
#include "motor.h"
#include "timer.h"

#define ENCA 2        //Encoder pinA
#define ENCB 3        //Encoder pinB
#define PWM 10        //motor PWM pin
#define IN2 23        //motor controller pin2
#define IN1 22        //motor controller pin1
#define BTN_PIN 7     //button pin

volatile int32_t posi = 0;

float Kp = 1.2; // 0.12; //Proportional gain // 0.12 // 0.18 
float Ki = 0.6; // 0.06; //Integral gain // 0.06 // 0.01 // 0.12
float Kd = 0.0; // 0.0; //Derivative gain
float motorSpeedMax = 60;

bool last_btn_state = HIGH; //Button state
int32_t current_pos = 0; //Current position

int last_state = 0;
enum states {IDLE, SET_TARGET, SET_START_POS, RUN, RUN_TO_START, READY_FOR_DROP};
int state = IDLE;

PID pid(Kp, Ki, Kd, -motorSpeedMax, motorSpeedMax);  //PID controller
Motor motor(PWM, IN1, IN2);  //DC motor
Timer StateTimer; //Timer

int target = 0;
int target_threshold = 100;
int start_pos = 0;
int dir = 1;
float u = 0;

VL53L0X range_sensor;
I2C_MPU6886 imu(I2C_MPU6886_DEFAULT_ADDRESS, Wire);

IPAddress ip(192, 168, 10, 240);
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

EthernetUDP udp_server;

char packet_buffer[UDP_TX_PACKET_MAX_SIZE];

void setup() 
{
  Serial.begin(115200);

  //Encoder
  pinMode (ENCA, INPUT);
  pinMode (ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  // DC MOTOR
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // BUTTON
  pinMode(BTN_PIN, INPUT_PULLUP);

  pid.reset();

  setState(SET_TARGET);

  Wire.begin();
  Ethernet.begin(mac, ip);
  delay(500);
  
  if(!initialize())
     while(true)
       delay(10);

  imu.begin();  
  range_sensor.startContinuous();

  udp_server.begin(8888);
  Serial.println("Setup complete");
}

bool initialize()
{
  range_sensor.setTimeout(500);
  if(!range_sensor.init())
  {
    Serial.println("Failed to detect and initialize range sensor!");
    return false;
  }
  
  if(Ethernet.hardwareStatus() == EthernetNoHardware) 
  {
    Serial.println("Ethernet shield was not found.");
    return false;
  }
  else if(Ethernet.hardwareStatus() == EthernetW5500) 
    Serial.println("Found W5500 ethernet shield");

  if(Ethernet.linkStatus() == LinkOFF) 
  {
    Serial.println("Ethernet::LinkOff: is the cable connected?");
    return false;
  }
  else if(Ethernet.linkStatus() == LinkON)
    Serial.println("Ethernet::LinkOn");
  return true;
}
  
void loop() 
{ 
  int packet_size = udp_server.parsePacket();
  if(packet_size)
  {
    float accel[3];
    float gyro[3];
    float t;
    float d;
  
    imu.getAccel(&accel[0], &accel[1], &accel[2]);      // Acceleration in X,Y,Z axis
    imu.getGyro(&gyro[0], &gyro[1], &gyro[2]);          // Gyro in X,Y,Z axis
    imu.getTemp(&t);                                    // Temperature
    d = range_sensor.readRangeContinuousMillimeters();  // Distance sensor

    String sensor_values;
    sensor_values.concat(accel[0]); sensor_values.concat(",");
    sensor_values.concat(accel[1]); sensor_values.concat(",");
    sensor_values.concat(accel[2]); sensor_values.concat(",");
    sensor_values.concat(d);

    udp_server.read(packet_buffer, UDP_TX_PACKET_MAX_SIZE);
    float motor_power = String(packet_buffer).toFloat();
    
    printVector3('A', accel);
    printVector3('G', gyro);
    printScalar('T', t);
    printScalar('D', d);
    Serial.print("MPP = "); Serial.println(motor_power);
    // printPackageMetaInfo(packet_size);
    
    udp_server.beginPacket(udp_server.remoteIP(), udp_server.remotePort());
    udp_server.write(sensor_values.c_str(), sensor_values.length());
    udp_server.endPacket();
  }

  // DRIVE MOTOR USING PID AND ESTIMATE
  bool btn_state = digitalRead(BTN_PIN);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    current_pos = posi;
  }

  switch (state)
  {
  case IDLE:
    motor.stop();
    if(btn_state && !last_btn_state){
      setState(SET_TARGET);
    }
    break;

  case SET_TARGET:
    motor.free();
    if(btn_state && !last_btn_state){
      current_pos = 0;
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
    Serial.print(millis());
    Serial.print(", ");
    Serial.print(target);
    Serial.print(", ");
    Serial.print(u);
    Serial.print(", ");
    Serial.println(current_pos);
    */
    if((current_pos <= start_pos + target_threshold && current_pos >= start_pos - target_threshold) or (btn_state && !last_btn_state)){
      motor.stop();
      setState(READY_FOR_DROP);
    }
    break;

  case RUN:
    float currentTime = millis();
    float ocillatingTarget = oscillate(currentTime) * start_pos;
    u = pid.update(current_pos, ocillatingTarget);
    motor.run(u);
    
    Serial.print(millis());
    Serial.print("TARGET: , ");
    Serial.print(target);
    Serial.print("OCILLATING TARGET: , ");
    Serial.print(ocillatingTarget);
    Serial.print("OCILLATION, ");
    Serial.print(oscillate(currentTime));
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
void printSensorInfo (float[3] &accel) {
  Serial.print(accel[0]);
  Serial.print(", ");
  Serial.print(accel[1]);
  Serial.print(", ");
  Serial.print(accel[2]);
}
*/
void printVector3(char label, float *vector)
{
  Serial.print(label); Serial.print(" = ");
  Serial.print(vector[0] > 0.0 ? " " : "");Serial.print(vector[0]);Serial.print(",    \t");
  Serial.print(vector[1] > 0.0 ? " " : "");Serial.print(vector[1]);Serial.print(",    \t");
  Serial.print(vector[2] > 0.0 ? " " : "");Serial.println(vector[2]);
}

void printScalar(char label, float scalar)
{
  Serial.print(label); Serial.print(" = ");
  Serial.print(scalar > 0.0 ? " " : "");Serial.println(scalar);
}

void printPackageMetaInfo(int packet_size)
{
  Serial.print("Received packet of size ");
  Serial.println(packet_size);
  Serial.print("From ");
  IPAddress remote = udp_server.remoteIP();
  for(int i = 0; i < 4; i++) 
  {
    Serial.print(remote[i], DEC);
    if(i < 3) 
      Serial.print(".");
  }
  Serial.print(", port ");
  Serial.println(udp_server.remotePort());
}

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
  Serial.print("State changed from ");
  Serial.print(stateStr(last_state));
  Serial.print(" to ");
  Serial.println(stateStr(state));
  Serial.print("DeltaT: ");
  Serial.println(StateTimer.get_deltaT());
  Serial.print("Current position: ");
  Serial.println(current_pos);
  Serial.println("");
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

float oscillate(float t) {
  const float w = PI/180 * t * 0.2;
  return sin(w);
}