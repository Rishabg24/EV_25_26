#include "Motor.h"
#include "PID.h"
#include <Encoder.h>
#include <Arduino.h>

PID::PID(float Kp, float Ki, float Kd, float wheelBase, float GR, float CPR){
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    aSpeed = 0.0;
    wBase = wheelBase;
    MPC = (PI*wBase)/(GR*CPR); 
}


int PID::output(Encoder& enc, int speed, float dt){
    float currDistance = enc.read()*MPC;
    float deltaDistance = currDistance - prevDistance;
    prevDistance = currDistance;
    float currSpeed = deltaDistance/dt;
    float error = speed*1.0-currSpeed;

    integral += error*dt;

    float derivative = (error - prev_error)/dt;
    prev_error = error;

    float output = Kp*error + Ki*integral + Kd*derivative;
    output = constrain(output, -255.f, 255.f);

    aSpeed = currSpeed;

    return output;
}

void PID::reset(){
    prevTime = 0;
    integral = 0.f;
    prevDistance = 0.f;
}