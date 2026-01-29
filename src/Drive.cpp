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

// EKF Assisted Drive Distance
void Drive::driveDistance(float distance, int speed, float theta)
{
    sensors_event_t a, g, temp;
    reset();
    Lcon.reset();
    Rcon.reset();
    lastHeadingError = 0.0f; // ADD: Reset derivative tracking

    float targetTheta = theta;
    unsigned long prevTime = micros();
    unsigned long now = micros();
    unsigned long start = micros();
    float dt = 0.0f;
    float lastLeftDist = 0.0f;
    float lastRightDist = 0.0f;
    float startX = ekf.getX();
    float startY = ekf.getY();
    float traveled = 0.0f;

    while (traveled < abs(distance))
    {
        now = micros();
        dt = (now - prevTime) / 1000000.f;
        if (dt >= 0.002f)
        {
            float leftDist = Lenc.read() * Lcon.MPC;
            float rightDist = Renc.read() * Rcon.MPC;
            float deltaLeft = leftDist - lastLeftDist;
            float deltaRight = rightDist - lastRightDist;

            ekf.predict(deltaLeft, deltaRight);

            lastLeftDist = leftDist;
            lastRightDist = rightDist;

            mpu.getEvent(&a, &g, &temp);
            float gyroZ = g.gyro.z - bias;
            ekf.update(gyroZ, dt);

            float dx = ekf.getX() - startX;
            float dy = ekf.getY() - startY;
            traveled = sqrt(dx * dx + dy * dy);

            if (traveled < 5.0f)
            {
                Lmotor.drive(Lcon.output(Lenc, speed, dt));
                Rmotor.drive(Rcon.output(Renc, speed, dt));
            }
            else
            {

                float speedRatio = speed_reference / (float)speed; // <1.0 at high speeds
                float Kc_scaled = Kc * speedRatio;
                float Ky_scaled = Ky * speedRatio;
                float Kcd_scaled = Kcd * (1.0f + (1.0f - speedRatio)); // MORE damping at high speed 
                float lateralError = (-sin(targetTheta) * dx) + (cos(targetTheta) * dy); // Normal control with corrections
                float sideCorrectionAngle = constrain(lateralError * Ky_scaled, -0.5f, 0.5f);
                float modifiedTargetTheta = targetTheta - sideCorrectionAngle;
                float headingError = ekf.normalizeAngle(modifiedTargetTheta - ekf.getTheta());

                // ADD DERIVATIVE TERM
                float headingDerivative = (headingError - lastHeadingError) / dt;
                lastHeadingError = headingError;

                // PD Control instead of just P
                float steeringCorrection = Kc_scaled * headingError + Kcd_scaled * headingDerivative;

                // Constrain to prevent saturation
                steeringCorrection = constrain(steeringCorrection, -100.0f, 100.0f);

                int RightMotorSpeed = speed + (int)steeringCorrection;
                int LeftMotorSpeed = speed - (int)steeringCorrection;

                Lmotor.drive(Lcon.output(Lenc, LeftMotorSpeed, dt));
                Rmotor.drive(Rcon.output(Renc, RightMotorSpeed, dt));
            }

            prevTime = now;
            Serial.print((now - start) / 1000.f);
            Serial.print(",");
            Serial.print(ekf.getX());
            Serial.print(",");
            Serial.print(ekf.getY());
            Serial.print(",");
            Serial.println(ekf.getTheta());
        }
    }
    stop();
}


// EKF Assisted Turn (if ever necessary)
void Drive::EKFturn(float theta_Radians, int speed)
{
    sensors_event_t a, g, temp;
    float targetTheta = theta_Radians; // TrapezoidManuever will pass in radian angles for turning
    reset();
    Lcon.reset();
    Rcon.reset();
    ekf.reset();
    unsigned long prevTime = millis();
    unsigned long now = millis();
    unsigned long start = millis();
    float dt = 0.0f;
    float LastLeftDist = 0.0f;
    float LastRightDist = 0.0f;

    int turnSpeed = (targetTheta > 0) ? speed : -speed; // Determine turn direction based on sign of targetTheta

    while (abs(ekf.getTheta()) < abs(targetTheta))
    {
        now = millis();
        dt = now - prevTime / 1000.f;
        if (dt >= 0.02f)
        {
            float leftDist = Lenc.read() * Lcon.MPC;
            float rightDist = Renc.read() * Rcon.MPC;

            float deltaLeft = leftDist - LastLeftDist;
            float deltaRight = rightDist - LastRightDist;

            ekf.predict(deltaLeft, deltaRight);

            mpu.getEvent(&a, &g, &temp);
            float gyroX = g.gyro.z - bias;

            ekf.update(gyroX, dt);

            int currentSpeed = turnSpeed;
            if (abs(ekf.getTheta()) > abs(targetTheta) * 0.8f)
            {
                currentSpeed = turnSpeed / 2;
            }

            // Tank turn: opposite wheel directions
            Lmotor.drive(Lcon.output(Lenc, currentSpeed, dt));
            Rmotor.drive(Rcon.output(Renc, -currentSpeed, dt));

            prevTime = now;

            // Debug
            Serial.print((now - start) / 1000.0f);
            Serial.print(",");
            Serial.print(ekf.getX());
            Serial.print(",");
            Serial.print(ekf.getY());
            Serial.print(",");
            Serial.print(ekf.getTheta() * 180.0f / PI);
            Serial.print(",");
            Serial.print(targetTheta * 180.0f / PI);
            Serial.print(Renc.read() * Rcon.MPC);
            Serial.println(Lenc.read() * Lcon.MPC);
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
    // Implementation of trapezoidal maneuver
    // This function would control the vehicle to move in a trapezoidal path
    // based on the provided lateral distance and total distance.
    // distance inputs in meters

    ekf.reset();
    reset();

    float d = totalDistance;
    float h = lateralDistance; // should be 0.5m

    float DiagonalDistance = (sqrt(((d / 4) * (d / 4)) + (h * h))) * 1000.f; // in mm
    float DistanceThroughGate = (d / 2.f) * 1000.f;                          // in mm

    float angle = atan2(h, d / 4); // angle in radians

    enum State
    {
        INIT_TURN,
        DIAGONAL_TRANSIT_1,
        ALIGN_GATE,
        DRIVE_GATE,
        EXIT_TURN,
        DIAGONAL_TRANSIT_2,
        COMPLETE
    };

    State currentState = INIT_TURN;

    while (currentState != COMPLETE)
    {
        switch (currentState)
        {
        case INIT_TURN:
            // Turn towards the first diagonal
            // Assuming a function turn(angle, speed) exists
            turn(angle * 180.0f / PI, speed); // converting to degrees for turn function
            currentState = DIAGONAL_TRANSIT_1;
            break;
        case DIAGONAL_TRANSIT_1:
            // Drive the diagonal distance
            driveDistance(DiagonalDistance, speed, angle);
            currentState = ALIGN_GATE;
            break;
        case ALIGN_GATE:
            // Align to face the gate directly
            turn(-angle * 180.0f / PI, speed); // negative angle to realign
            currentState = DRIVE_GATE;
            break;
        case DRIVE_GATE:
            // Drive through the gate
            driveDistance(DistanceThroughGate, speed, 0.0f);
            currentState = EXIT_TURN;
            break;
        case EXIT_TURN:
            // Turn to face the exit diagonal
            turn(-angle * 180.0f / PI, speed);
            currentState = DIAGONAL_TRANSIT_2;
            break;
        case DIAGONAL_TRANSIT_2:
            // Drive the second diagonal distance
            driveDistance(DiagonalDistance, speed, -angle);
            currentState = COMPLETE;
            break;
        case COMPLETE:
            // Maneuver complete
            stop();
            break;
        }
    }
    stop(); // just in case
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