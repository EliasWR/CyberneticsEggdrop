#ifndef MOTOR_h
#define MOTOR_h

#include <Arduino.h>

class Motor{

    public:
        Motor(int pwm_pin, int in1_pin, int in2_pin){
            pwmPin = pwm_pin;
            in1Pin = in1_pin;
            in2Pin = in2_pin;
        }

        void run(float u)
        {
            int dir;
            if(u > 0){
                dir = UP;
            }
            else{
                dir = DOWN;
            }

            run((0 < u) - (0 > u), abs(int(u)));
        }

        void run(int dir, int pwmVal){

            analogWrite(pwmPin, pwmVal);
            if(dir == 1){
                digitalWrite(in1Pin, LOW);
                digitalWrite(in2Pin, HIGH);
            }
            else if(dir == -1){
                digitalWrite(in1Pin, HIGH);
                digitalWrite(in2Pin, LOW);
            }
            else{
                digitalWrite(in1Pin, LOW);
                digitalWrite(in2Pin, LOW);
            }
        }

        void free(){
            analogWrite(pwmPin, 0);
            digitalWrite(in1Pin, LOW);
            digitalWrite(in2Pin, LOW);
        }

        void stop(){
            analogWrite(pwmPin, 0);
            digitalWrite(in1Pin, HIGH);
            digitalWrite(in2Pin, HIGH);
            delay(300);
            free(); // To prevent the motor from overheating
        }

        static const int DOWN = -1;
        static const int UP = 1;
        static const int FREE = 0;

    private:
        int pwmVal = 0;
        int pwmPin;
        int in1Pin;
        int in2Pin;
};



#endif // MOTOR_h
