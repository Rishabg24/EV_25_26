#include "Motor.h"
#include <Encoder.h>

Motor::Motor(uint8_t in1, uint8_t in2, uint8_t ena){
    IN1 = in1;
    IN2 = in2;
    ENA = ena;
}

void Motor::begin(){
    Serial.begin(9600);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
}

void Motor::drive(int speed){
    if(speed>0){
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA,speed);
    }else{
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, -1*speed);
    }
}

