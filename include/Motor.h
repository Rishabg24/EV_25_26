#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>
#include <Encoder.h>

class Motor{
    public:

        Motor(uint8_t in1, uint8_t in2, uint8_t ena);
        float distance();
        long output();
        void drive(int PWM);
        void begin();
        void movePID(Encoder enc);

    private:

        uint8_t IN1; //IN1 pin
        uint8_t IN2; //IN2 pin
        uint8_t ENA; //ENA pin

        float Kp;
        float Ki;
        float Kd;

};


#endif