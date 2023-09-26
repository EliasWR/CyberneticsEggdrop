#ifndef PID_h
#define PID_h

#include <Arduino.h>

class PID{

    
    public:
        PID(float kp, float ki, float kd, int min, int max): _kp(kp), _ki(ki), _kd(kd), _min(min), _max(max){
        };
        PID(float kp, float ki, float kd): _kp(kp), _ki(ki), _kd(kd), _min(-255), _max(255){
        };

        float update(float pos, float targ){
            float deltaT = calculate_deltaT();

            float e = (targ - pos);
            float de_dt = (e - _eprev) / deltaT;  //Derivative
            _eintegral = _eintegral + e * deltaT; //Integral

            float u = _kp*e + _ki*_eintegral + _kd*de_dt;  //Control signal
            u = limit(u, _min, _max); //Capping
            
            _eprev = e;
            return u;
        };
        void reset(){
            _eprev = 0;
            _eintegral = 0;
        };

    private:
        float _kp;
        float _ki;
        float _kd;
        float _min;
        float _max;
        float _eprev = 0;
        float _eintegral = 0;
        unsigned long prev_time;


        float limit(float val, float min, float max){
            if(val < min){
                val = min;
            }
            if(val > max){
                val = max;
            }
            return val;
        };

        float calculate_deltaT(){
            unsigned long cur_time =  millis();
            float deltaT = ((float) (cur_time - prev_time))/(1.0e6);
            prev_time = cur_time; 
            return deltaT;
        }
    };

#endif
