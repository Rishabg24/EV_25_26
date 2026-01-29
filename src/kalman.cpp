#include <kalman.h>
#include <Arduino.h>
#include <math.h>

EKFState::EKFState(float wb, float wr)
{
    wheelbase = wb;
    wheel_radius = wr;

    x = 0.0f;
    y = 0.0f;
    theta = 0.0f;

    // Initialize process noise parameters. " How much do we trust the model"
    Q_x = 0.01f;     // TUNE THESE VALUES
    Q_y = 0.01f;     // TUNE THESE VALUES
    Q_theta = 0.01f; // TUNE THESE VALUES

    // Initialize measurement noise parameter. " How much do we trust the gyro"
    R_gyro = 0.01f; // TUNE THIS VALUE

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            P[i][j] = 0.0f;
        }
    }

    // Initialize covariance matrix P with small values. " How confident are we in initial state"
    P[0][0] = 0.01f;  // TUNE THESE VALUES
    P[1][1] = 0.01f;  // TUNE THESE VALUES
    P[2][2] = 0.001f; // TUNE THESE VALUES

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            Q[i][j] = 0.0f;
        }
    }

    // Initialize process noise covariance matrix Q
    Q[0][0] = Q_x;
    Q[1][1] = Q_y;
    Q[2][2] = Q_theta;
}

float EKFState::normalizeAngle(float angle)
{
    while (angle > PI)
        angle -= 2.0f * PI;
    while (angle < -PI)
        angle += 2.0f * PI;
    return angle;
}

void EKFState::matrixMultiply(float A[3][3], float B[3][3], float result[3][3])
{

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result[i][j] = 0.0f;
            for (int k = 0; k < 3; k++)
            {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void EKFState::predict(float d_left, float d_right)
{
    // KINEMATICS: Convert encoder ticks to center displacement and heading change
    float d_center = (d_left + d_right) / 2.0f;
    float d_theta = (d_right - d_left) / wheelbase;
    float half_theta = theta + (d_theta / 2.0f);

    // STATE ESTIMATE UPDATE: Moving the robot in the world
    if (abs(d_theta) < 1e-6) // Straight line case (prevents division by zero)
    {
        x += d_center * cos(half_theta);
        y += d_center * sin(half_theta);
        theta += d_theta;
    }
    else // Arc or trapezoid manuever case
    {
        float radius = d_center / d_theta;
        x += d_center * cos(half_theta); // radius * (sin(theta_old + d_theta) - sin(theta_old));
        y += d_center * sin(half_theta); // radius * (-cos(theta_old + d_theta) + cos(theta_old));
        theta += d_theta;
    }
    theta = normalizeAngle(theta);

    // JACOBIAN (F): Linearizing the physics model
    // This matrix describes how a small error in the current state (x, y, theta)
    // propagates to the next state.

    float F[3][3];
    F[0][0] = 1.0f;
    F[0][1] = 0.0f;

    if (abs(d_theta) < 1e-6) // Straight line case
    {
        F[0][2] = -d_center * sin(half_theta);
        F[1][2] = d_center * cos(half_theta);
    }
    else // Arc or trapezoid manuever case
    {
        // float radius = d_center / d_theta;
        // F[0][2] = radius * (cos(half_theta + d_theta) - cos(half_theta));
        // F[1][2] = radius * (sin(half_theta + d_theta) - sin(half_theta));

        F[0][2] = -d_center * sin(half_theta); // Same as straight!
        F[1][2] = d_center * cos(half_theta);  // Same as straight!
    }
    F[1][0] = 0.0f;
    F[1][1] = 1.0f;
    F[2][0] = 0.0f;
    F[2][1] = 0.0f;
    F[2][2] = 1.0f;

    // COVARIANCE PREDICTION: P = F*P*F' + Q
    // We increase our uncertainty because moving always introduces noise.
    float FP[3][3];
    matrixMultiply(F, P, FP);

    float P_new[3][3];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            P_new[i][j] = Q[i][j]; // Start with process noise
            for (int k = 0; k < 3; k++)
            {
                P_new[i][j] += FP[i][k] * F[j][k]; // Add the FPF' term to propagate uncertainty
            }
        }
    }

    // Update the global P matrix
    memcpy(P, P_new, sizeof(P));
}

void EKFState::update(float gyro_z, float dt)
{
    // INNOVATION: Compare what we thought happened to what the sensor says
    float theta_pred = theta;

    // We treat the integrated gyro as a measurement of our heading
    float theta_meas = theta + gyro_z * dt;
    theta_meas = normalizeAngle(theta_meas);
    float innovation = normalizeAngle(theta_meas - theta_pred);

    // RESIDUAL COVARIANCE: S = H*P*H' + R
    // Since we only measure theta, H is effectively [0, 0, 1]
    float S = P[2][2] + R_gyro;

    // KALMAN GAIN (K): How much do we trust the sensor vs. the encoders?
    // K = P * H' * inv(S)

    float K[3];
    K[0] = P[0][2] / S; // Impact of heading error on X estimate
    K[1] = P[1][2] / S; // Impact of heading error on Y estimate
    K[2] = P[2][2] / S; // Impact of heading error on Theta estimate

    // STATE CORRECTION: Adjust x, y, theta based on the gyro's feedback
    x += K[0] * innovation;
    y += K[1] * innovation;
    theta += K[2] * innovation;
    theta = normalizeAngle(theta);

    // COVARIANCE UPDATE: P = (I - KH)P
    // This shrinks our uncertainty because we just got new data.
    float IKH[3][3];
    IKH[0][0] = 1.0f;
    IKH[0][1] = 0.0f;
    IKH[0][2] = -K[0];
    IKH[1][0] = 0.0f;
    IKH[1][1] = 1.0f;
    IKH[1][2] = -K[1];
    IKH[2][0] = 0.0f;
    IKH[2][1] = 0.0f;
    IKH[2][2] = 1.0f - K[2];

    float P_new[3][3];
    matrixMultiply(IKH, P, P_new);

    // Update the global P matrix
    memcpy(P, P_new, sizeof(P));
}

void EKFState::reset()
{
    x = 0.0f;
    y = 0.0f;
    theta = 0.0f;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            P[i][j] = 0.0f;
        }
    }

    P[0][0] = 0.001f;
    P[1][1] = 0.001f;
    P[2][2] = 0.001f;
}

void EKFState::printState()
{
    Serial.print("X: ");
    Serial.print(x);
    Serial.print(" Y: ");
    Serial.print(y);
    Serial.print(" Theta: ");
    Serial.println(theta);
}
