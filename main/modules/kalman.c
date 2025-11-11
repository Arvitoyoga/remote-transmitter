#include "kalman.h"

float Q[NUM_VAR] = {0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8}; // Process noise
float R[NUM_VAR] = {2, 2, 2, 2, 2, 2, 2};             // Measurement noise
float X[NUM_VAR] = {0, 0, 0, 0, 0, 0, 2};             // State estimate
float P[NUM_VAR] = {1, 1, 1, 1, 1, 1, 1};             // Estimation error covariance
float K[NUM_VAR];                                  // Kalman gain

float kalmanFilterMulti(float measurement, int index) {
    P[index] = P[index] + Q[index];
    K[index] = P[index] / (P[index] + R[index]);
    X[index] = X[index] + K[index] * (measurement - X[index]);
    P[index] = (1 - K[index]) * P[index];
    return X[index];
}
