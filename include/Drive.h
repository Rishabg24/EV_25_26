#ifndef DRIVE_H
#define DRIVE_H
#include <Arduino.h>
#include <Encoder.h>
#include "PID.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <kalman.h>

class Drive{
    public:
        
        Drive(PID& Lcon, PID& Rcon, Motor& Lmotor,Motor& Rmotor, Encoder& Lenc, Encoder& Renc, Adafruit_MPU6050& mpu6050, float wheelBase, float wheelDiameter): 
            Lcon(Lcon),
            Rcon(Rcon),
            Rmotor(Rmotor),
            Lmotor(Lmotor),
            Lenc(Lenc),
            Renc(Renc),
            mpu(mpu6050),      
            wBase(wheelBase),         // Initialize wBase here
            wDiameter(wheelDiameter),
            ekf(wheelBase, wheelDiameter / 2.0f)
    {
        
    }
        void driveDistance(float distance, int speed, float theta);
        void stop();
        void reset();
        void turnR(int speed);
        void turnL(int speed);
        void sTurnR(int speed);
        void sTurnL(int speed);
        void turn(float degree, int speed);
        void EKFturn(float degree, int speed);
        void trapezoidManuever(float lateralDistance, float totalDistance, int speed);
        void begin();
        void accel(int targetSpeed, int accelRate);
        void decel(int targetSpeed, int decelRate);
        void driveStraightMission(float distance, int speed);

    private:

        PID& Lcon;
        PID& Rcon;
        Motor& Rmotor;
        Motor& Lmotor;
        Encoder& Lenc;
        Encoder& Renc;
        Adafruit_MPU6050& mpu;


        unsigned long now;
        float wBase;
        float wDiameter;
        float bias = 0.f;

        float Kp = 0.0001f; 
        float Kc = 0.5f; //12.0f;  // heading gain constant
        float Ky = 0.30f;   // lateral gain constant
        float Kcd = 0.0f; //6.0f // lateral derivative gain constant
        float lastHeadingError = 0.f;
        float speed_reference = 515.f; // reference speed for straight drive



        uint8_t startPWM(int linSpeed);

        EKFState ekf;

};

#endif