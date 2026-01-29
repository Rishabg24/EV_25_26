#ifndef KALMAN_H
#define KALMAN_H
#include <Arduino.h>

class EKFState{
    private:
        // Process noise parameters
        float Q_x;
        float Q_y;
        float Q_theta;
        
        // Measurement noise
        float R_gyro;
        
        // EV geometry
        float wheelbase;
        float wheel_radius;
        
        // State vector
        float x;
        float y;
        float theta;
        
        // Matrices
        float P[3][3];
        float Q[3][3];

        // Helper functions
        void calculateJacobian(float d_left, float d_right, float F[3][3]);
        void matrixMultiply(float A[3][3], float B[3][3], float result[3][3]);
        
    public:
        EKFState(float wb, float wr);  // Constructor with geometry (wheelbase, wheel_radius)
        
        void predict(float delta_left, float delta_right);
        void update(float gyro_z, float dt);
        float normalizeAngle(float angle);  // Wrap to [-PI, PI]
        void reset();
        
        // Getters
        float getX() { return x; }
        float getY() { return y; }
        float getTheta() { return theta; }
        
        // Debug
        void printState();
};

#endif