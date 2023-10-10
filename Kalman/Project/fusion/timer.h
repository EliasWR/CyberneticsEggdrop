#ifndef TIMER_h
#define TIMER_h

#include <Arduino.h>

class Timer{
    
        public:
            Timer(){
                reset();
            };
    
            void reset(){
                prev_time = millis();
            };

            void reset(int delay){
                prev_time = millis();
                this->next_time = prev_time + delay;
            };
    
            float get_deltaT(){
                unsigned long cur_time =  millis();
                float deltaT = ((float) (cur_time - prev_time))/(1.0e3);
                prev_time = cur_time; 
                return deltaT;
            }

            void expired(){
                return millis() > next_time;
            }

    
        private:
            unsigned long prev_time;
            unsigned long next_time;
};



#endif // TIMER_H