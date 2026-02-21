#include <Arduino.h>
#include "Motor.h"
#include "PID.h"
#include "Drive.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Kalman.h>

Adafruit_MPU6050 mpu;
Encoder Rencoder(3 /*Encoder Pin A*/, 5 /*Encoder Pin B*/);
Motor Rmotor(8 /*IN1*/, 9 /*IN2*/, 11 /*ENA*/);
Encoder Lencoder(2 /*Encoder Pin A*/, 4 /*Encoder Pin B*/);
Motor Lmotor(6 /*IN3*/ , 7 /*IN4*/ ,10 /*ENB*/);
PID right(.01f/*Kp*/, .2f/*Ki*/, 0.0f/*Kd*/, 60.f/*Diameter of the wheels*/, 100.37f /*Gear Ratio of Motor*/, 12 /*CPR*/);
PID left(.01f/*Kp*/, .232f/*Ki*/, 0.0f/*Kd*/, 60.f/*Diameter of the wheels*/, 100.37f /*Gear Ratio of Motor*/, 12 /*CPR*/);
uint8_t butxtonState;

Drive drive(left, right, Lmotor, Rmotor, Lencoder, Rencoder, mpu, 106 /* The distance between the wheels*/, 60 /* The diameter of the wheels*/);
bool lastState = HIGH;

void setup() {
  pinMode(12, INPUT_PULLUP);
  drive.begin();
  Serial.begin(9600);
}


void loop() {
  bool currentState = digitalRead(12);
  bool runState = false;
  if (lastState == HIGH && currentState == LOW) {
    Serial.println("Button pressed");
    runState = !runState;
  }
 
  lastState = currentState;
  delay(20);
  if(runState){
    // change to trapezoidManuever for trapezoid maneuver
    // lateral distance should be 0.5m to test, and total distance can be whatever
    // speed needs to be calculated based on the time we want it to take. 
    drive.driveDistance(9800,595, 0.0f); // driveDistance in mm, speed in PWM value, theta in radians
    // Serial.print(Lencoder.read());
    // Serial.print(",");
    // Serial.println(Rencoder.read());
    // VERY STRAIGHT WITH 515 PWM, but it does drift to the left ever so slightly. 
    // Could change above to driveStraightMission( distance, speed), this ensures that everything is reset properly for a new run
    // drive.driveStraightMission(1000, 700); // distance in mm, speed in PWM value
    // drive.trapezoidManuever(0.5f, 2.0f, 700);
    drive.stop();
    runState = false;
  }
}


/* SciOly EV Debug Script 
   Testing Motors and Encoders based on your pinout
*/
// m