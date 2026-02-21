#include "Drive.h"
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <kalman.h>

void Drive::begin()
{
    Serial.begin(9600);
    Rmotor.begin();
    Lmotor.begin();
    if (!mpu.begin())
    {
        Serial.println("Failed to find MPU6050 chip");
        while (1)
        {
            delay(10);
        }
    }
    Serial.println("MPU6050 Found!");
    // set accelerometer range to +-8G
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

    // set gyro range to +- 500 deg/s
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);

    // set filter bandwidth to 21 Hz
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    for (int i = 0; i < 1000; i++)
    {
        bias += g.gyro.z;
        delay(1);
    }

    bias = bias / 1000.f;
    Serial.print("Gyro Calibration done");

    delay(100);
}
// EKF Assisted Drive Distance with Decoupled Control Loops
void Drive::driveDistance(float distance, int speed, float theta)
{
    sensors_event_t a, g, temp;
    reset();
    Lcon.reset();
    Rcon.reset();
    lastHeadingError = 0.0f;

    float targetTheta = theta;

    // Separate timing trackers
    unsigned long lastNavTime = micros();
    unsigned long prevMotorTime = micros();
    unsigned long start = micros();

    float lastLeftDist = 0.0f;
    float lastRightDist = 0.0f;
    float startX = ekf.getX();
    float startY = ekf.getY();
    float traveled = 0.0f;

    // Persistent targets updated by Navigation loop, read by Motor loop
    int currentLeftTarget = speed;
    int currentRightTarget = speed;

    while (traveled < abs(distance))
    {
        unsigned long now = micros();

        // ==========================================================
        // 1. NAVIGATION & SENSOR LOOP (Run at ~50Hz / 20ms)
        // This loop determines WHERE the robot is and what speed it SHOULD go.
        // ==========================================================
        if (now - lastNavTime >= 20000)
        {
            float dt_nav = (now - lastNavTime) / 1000000.f;
            lastNavTime = now;

            // --- EKF Update (User Logic) ---
            float leftDist = Lenc.read() * Lcon.MPC;
            float rightDist = Renc.read() * Rcon.MPC;
            float deltaLeft = leftDist - lastLeftDist;
            float deltaRight = rightDist - lastRightDist;
            ekf.predict(deltaLeft, deltaRight);
            lastLeftDist = leftDist;
            lastRightDist = rightDist;

            mpu.getEvent(&a, &g, &temp);
            float gyroZ = g.gyro.z - bias;
            ekf.update(gyroZ, dt_nav);

            float dx = ekf.getX() - startX;
            float dy = ekf.getY() - startY;
            traveled = sqrt(dx * dx + dy * dy);

            // --- Correction Logic (User Terms) ---
            if (traveled < 5.0f)
            {
                currentLeftTarget = speed;
                currentRightTarget = speed;
            }
            else
            {
                // Scaled Gains & Lateral Error logic
                float speedRatio = speed / speed_reference;
                float Kc_scaled = Kc * speedRatio;
                float Ky_scaled = Ky * speedRatio;
                float Kcd_scaled = Kcd * (1.0f + (1.0f - speedRatio));

                float lateralError = (-sin(targetTheta) * dx) + (cos(targetTheta) * dy);
                float sideCorrectionAngle = constrain(lateralError * Ky_scaled, -0.5f, 0.5f);
                float modifiedTargetTheta = targetTheta - sideCorrectionAngle;
                float headingError = ekf.normalizeAngle(modifiedTargetTheta - ekf.getTheta());

                // Derivative Term
                float headingDerivative = (headingError - lastHeadingError) / dt_nav;
                lastHeadingError = headingError;

                float steeringCorrection = Kc_scaled * headingError + Kcd_scaled * headingDerivative;
                steeringCorrection = constrain(steeringCorrection, -100.0f, 100.0f);

                currentRightTarget = speed + (int)steeringCorrection;
                currentLeftTarget = speed - (int)steeringCorrection;
            }

            // Standard Debugging
            Serial.print((now - start) / 1000.f);
            Serial.print(",");
            Serial.print(ekf.getX());
            Serial.print(",");
            Serial.print(ekf.getY());
            Serial.print(",");
            Serial.println(ekf.getTheta());
        }

        // ==========================================================
        // 2. MOTOR CONTROL LOOP (Run as fast as possible)
        // This loop applies the PID to reach the targets set above.
        // ==========================================================
        
        unsigned long motorNow = micros(); // fresh timestamp
        float dt_motor = (motorNow - prevMotorTime) / 1000000.f;
        if (dt_motor >= 0.01f)
        {
            Lmotor.drive(Lcon.output(Lenc, currentLeftTarget, dt_motor));
            Rmotor.drive(Rcon.output(Renc, currentRightTarget, dt_motor));
            prevMotorTime = motorNow;
        }
    }
    stop();
}

void Drive::EKFturn(float targetTheta, int speed)
{
    sensors_event_t a, g, temp;
    // We do NOT reset the EKF here so it maintains its global orientation
    Lcon.reset();
    Rcon.reset();

    unsigned long lastNavTime = micros();
    unsigned long prevMotorTime = micros();

    int turnDirection = (targetTheta > ekf.getTheta()) ? 1 : -1;
    int currentLeftTarget = speed * turnDirection;
    int currentRightTarget = -speed * turnDirection;

    // Turn until we reach the global target angle
    while (abs(ekf.normalizeAngle(targetTheta - ekf.getTheta())) > 0.02)
    {
        unsigned long now = micros();

        // 50Hz Navigation/EKF Loop
        if (now - lastNavTime >= 20000)
        {
            float dt_nav = (now - lastNavTime) / 1000000.f;
            lastNavTime = now;

            float leftDist = Lenc.read() * Lcon.MPC;
            float rightDist = Renc.read() * Rcon.MPC;
            // No lastDist tracking here for simplicity, but EKF needs deltas
            ekf.predict(0, 0); // Rotation only, small simplification

            mpu.getEvent(&a, &g, &temp);
            ekf.update(g.gyro.z - bias, dt_nav);

            // Optional: Slow down as we approach the target
            if (abs(ekf.normalizeAngle(targetTheta - ekf.getTheta())) < 0.2)
            {
                currentLeftTarget = (speed / 2) * turnDirection;
                currentRightTarget = -(speed / 2) * turnDirection;
            }
        }

        // Fast Motor Loop
        float dt_motor = (now - prevMotorTime) / 1000000.f;
        if (dt_motor >= 0.001f)
        {
            Lmotor.drive(Lcon.output(Lenc, currentLeftTarget, dt_motor));
            Rmotor.drive(Rcon.output(Renc, currentRightTarget, dt_motor));
            prevMotorTime = now;
        }
    }
    stop();
}

void Drive::driveStraightMission(float distance, int speed)
{
    // Reset everything because we are starting a NEW run from the start line
    ekf.reset();
    reset(); // Reset motor encoders

    // 2. Drive at heading 0
    driveDistance(distance, speed, 0.0f);
}

void Drive::trapezoidManuever(float lateralDistance, float totalDistance, int speed)
{
    // 1. Math Setup
    float d = totalDistance;
    float h = lateralDistance;

    // Geometry calculations
    float diagDist = (sqrt(pow(d / 4.0f, 2) + pow(h, 2))) * 1000.f; // mm
    float gateDist = (d / 2.0f) * 1000.f;                           // mm
    float angleRad = atan2(h, d / 4.0f);                            // radians

    // 2. Reset System
    ekf.reset();
    reset();

    // 3. State Machine Definition
    enum State
    {
        TURN_TO_DIAG_1,
        DRIVE_DIAG_1,
        ALIGN_GATE,
        DRIVE_GATE,
        TURN_TO_DIAG_2,
        DRIVE_DIAG_2,
        DONE
    };

    State currentState = TURN_TO_DIAG_1;
    bool missionComplete = false;

    Serial.println("Starting Trapezoid Mission");

    while (!missionComplete)
    {
        switch (currentState)
        {
        case TURN_TO_DIAG_1:
            Serial.println("State: Turn to Diag 1");
            EKFturn(angleRad, speed);
            currentState = DRIVE_DIAG_1;
            break;

        case DRIVE_DIAG_1:
            Serial.println("State: Drive Diag 1");
            // We pass 'angleRad' so the robot corrects itself to maintain that heading
            driveDistance(diagDist, speed, angleRad);
            currentState = ALIGN_GATE;
            break;

        case ALIGN_GATE:
            Serial.println("State: Align to Gate");
            // Turn back to 0 (Straight)
            EKFturn(0.0f, speed);
            currentState = DRIVE_GATE;
            break;

        case DRIVE_GATE:
            Serial.println("State: Drive Through Gate");
            driveDistance(gateDist, speed, 0.0f);
            currentState = TURN_TO_DIAG_2;
            break;

        case TURN_TO_DIAG_2:
            Serial.println("State: Turn to Diag 2");
            // Turn to negative angle
            EKFturn(-angleRad, speed);
            currentState = DRIVE_DIAG_2;
            break;

        case DRIVE_DIAG_2:
            Serial.println("State: Drive Diag 2");
            driveDistance(diagDist, speed, -angleRad);
            currentState = DONE;
            break;

        case DONE:
            Serial.println("Mission Complete");
            stop();
            missionComplete = true;
            break;
        }

        // Optional: Add a small delay between states to let physics settle
        delay(100);
    }
}

void Drive::stop()
{
    // Initial cut of forward power
    Rmotor.drive(0);
    Lmotor.drive(0);

    // Apply a brief reverse "counter-pulse"

    Rmotor.drive(-200);
    Lmotor.drive(-200);

    // The Duration: Hold the reverse pulse for a few milliseconds
    // This value needs to be tuned based on your vehicle's weight/speed
    delay(50);

    // 4. Final Stop: Cut power so it doesn't keep moving backward
    Rmotor.drive(0);
    Lmotor.drive(0);
}

void Drive::reset()
{
    Renc.write(0);
    Lenc.write(0);
}

uint8_t Drive::startPWM(int linSpeed)
{
    float outputI = 40.f - 40.f * log(1 - abs(linSpeed) / 550.f);
    uint8_t output = constrain((int)outputI, 40, 230);
    return output;
}

void Drive::turnL(int speed)
{
    reset();
    Lcon.reset();
    Rcon.reset();
    unsigned long prevTime = millis();
    float currDistanceLeft = 0.f;
    float currDistanceRight = 0.f;
    unsigned long now = millis();
    unsigned long start = millis();
    float dt = (now - prevTime) / 1000.f;
    while (currDistanceLeft < (250 + wBase / 2.f) * (PI / 2.f) && currDistanceRight < (250 - wBase / 2.f) * (PI / 2.f))
    {
        dt = (now - prevTime) / 1000.f;
        if (dt >= 0.02f)
        {
            Lmotor.drive(Lcon.output(Lenc, speed * 1.54, dt));
            Rmotor.drive(Rcon.output(Renc, speed, dt));
            prevTime = now;
            currDistanceLeft = Lenc.read() * Lcon.MPC;
            currDistanceRight = Renc.read() * Rcon.MPC;
            Serial.print(now - start);
            Serial.print(",");
            Serial.print(Lcon.aSpeed);
            Serial.print(",");
            Serial.print(Rcon.aSpeed);
            Serial.print(",");
            Serial.print(speed * 1.54f);
            Serial.print(",");
            Serial.print(speed);
            Serial.print(",");
            Serial.print(currDistanceLeft);
            Serial.print(",");
            Serial.println(currDistanceRight);
        }
        now = millis();
    }
    stop();
}

void Drive::turnR(int speed)
{
    reset();
    Lcon.reset();
    Rcon.reset();
    unsigned long prevTime = millis();
    float currDistanceLeft = 0.f;
    float currDistanceRight = 0.f;
    unsigned long now = millis();
    unsigned long start = millis();
    float dt = (now - prevTime) / 1000.f;
    while (currDistanceLeft < (250 - wBase / 2.f) * (PI / 2.f) && currDistanceRight < (250 + wBase / 2.f) * (PI / 2.f))
    {
        dt = (now - prevTime) / 1000.f;
        if (dt >= 0.02f)
        {
            Lmotor.drive(Lcon.output(Lenc, speed * 1.54f, dt));
            Rmotor.drive(Rcon.output(Renc, speed, dt));
            prevTime = now;
            currDistanceLeft = Lenc.read() * Lcon.MPC;
            currDistanceRight = Renc.read() * Rcon.MPC;
            Serial.print(now - start);
            Serial.print(",");
            Serial.print(Lcon.aSpeed);
            Serial.print(",");
            Serial.print(Rcon.aSpeed);
            Serial.print(",");
            Serial.print(speed);
            Serial.print(",");
            Serial.print(speed * 1.54f);
            Serial.print(",");
            Serial.print(currDistanceLeft);
            Serial.print(",");
            Serial.println(currDistanceRight);
        }
        now = millis();
    }
    stop();
}

void Drive::sTurnL(int speed)
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float heading = 0.f;
    reset();
    Lcon.reset();
    Rcon.reset();
    unsigned long prevTime = millis();
    unsigned long now = millis();
    unsigned long start = millis();
    float dt = (now - prevTime) / 1000.f;

    while (abs(heading) + 4 * PI / 180.f < PI / 2)
    {
        mpu.getEvent(&a, &g, &temp);
        dt = (now - prevTime) / 1000.f;
        if (abs(heading) > (PI / 2.f) * 0.8)
        {
            speed = constrain(speed / 2, 50, speed);
        }
        if (dt >= 0.02f)
        {
            Lmotor.drive(Lcon.output(Lenc, speed * -1, dt));
            Rmotor.drive(Rcon.output(Renc, speed, dt));
            prevTime = now;
            Serial.print(now - start);
            Serial.print(",");
            Serial.print(Lcon.aSpeed);
            Serial.print(",");
            Serial.print(Rcon.aSpeed);
            Serial.print(",");
            Serial.print(speed);
            Serial.print(",");
            Serial.print(speed * -1);
            Serial.print(",");
            Serial.println(heading);
        }
        now = millis();
        heading += (g.gyro.z - bias) * dt;
    }
    stop();
}

void Drive::sTurnR(int speed)
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float heading = 0.f;
    reset();
    Lcon.reset();
    Rcon.reset();
    unsigned long prevTime = millis();
    unsigned long now = millis();
    unsigned long start = millis();
    float dt = (now - prevTime) / 1000.f;

    while (abs(heading) + 4 * PI / 180.f < PI / 2)
    {
        mpu.getEvent(&a, &g, &temp);
        dt = (now - prevTime) / 1000.f;
        if (abs(heading) > (PI / 2.f) * 0.8)
        {
            speed = constrain(speed / 2, 50, speed);
        }
        if (dt >= 0.02f)
        {
            Lmotor.drive(Lcon.output(Lenc, speed, dt));
            Rmotor.drive(Rcon.output(Renc, speed * -1, dt));
            prevTime = now;
            Serial.print(now - start);
            Serial.print(",");
            Serial.print(Lcon.aSpeed);
            Serial.print(",");
            Serial.print(Rcon.aSpeed);
            Serial.print(",");
            Serial.print(speed);
            Serial.print(",");
            Serial.print(speed * -1);
            Serial.print(",");
            Serial.println(heading);
        }
        now = millis();
        heading += (g.gyro.z - bias) * dt;
    }
    stop();
}

void Drive::turn(float degree, int speed)
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float heading = 0.f;
    reset();
    Lcon.reset();
    Rcon.reset();
    unsigned long prevTime = millis();
    unsigned long now = millis();
    unsigned long start = millis();
    float dt = (now - prevTime) / 1000.f;
    if (degree < 0)
    {
        speed = -1 * speed;
    }

    while (abs(heading) + 4 / 90 * degree * PI / 180.f < degree * PI / 180.f)
    {
        mpu.getEvent(&a, &g, &temp);
        now = millis();
        dt = (now - prevTime) / 1000.f;
        if (abs(heading) > degree * (PI / 180.f) * 0.8)
        {
            speed = constrain(speed / 2, 50, speed);
        }
        if (dt >= 0.02f)
        {
            Lmotor.drive(Lcon.output(Lenc, speed, dt));
            Rmotor.drive(Rcon.output(Renc, speed * -1, dt));
            prevTime = now;
            Serial.print(now - start);
            Serial.print(",");
            Serial.print(Lcon.aSpeed);
            Serial.print(",");
            Serial.print(Rcon.aSpeed);
            Serial.print(",");
            Serial.print(speed);
            Serial.print(",");
            Serial.print(speed * -1);
            Serial.print(",");
            Serial.println(heading);
        }
        heading += (g.gyro.z - bias) * dt;
    }
    stop();
}

void Drive::accel(int targetSpeed, int accelRate)
{
    int tspeed = 0;
    unsigned long start = millis();
    unsigned long prevTime = millis();
    unsigned long now = millis();
    float dt = (now - prevTime) / 1000.f;
    float adt = (now - prevTime) / 1000.f;
    int conAccel = accelRate;
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    while (Lcon.aSpeed < targetSpeed && Rcon.aSpeed < targetSpeed)
    {
        now = millis();
        dt = (now - prevTime) / 1000.f;
        if (dt >= 0.02f)
        {
            if (adt >= 0.6f)
            {
                conAccel = accelRate;
                adt = 0.f;
            }
            else
            {
                conAccel = 0;
            }
            Lmotor.drive(Lcon.output(Lenc, tspeed, dt));
            Rmotor.drive(Rcon.output(Renc, tspeed, dt));
            prevTime = now;
            tspeed += accelRate * dt;
            Serial.print((now - start) / 1000.f);
            Serial.print(",");
            Serial.print(Lcon.aSpeed);
            Serial.print(",");
            Serial.print(Rcon.aSpeed);
            Serial.print(",");
            Serial.println(tspeed);
        }
    }
}