#ifndef __KALMAN_H
#define __KALMAN_H

/** Implementation of 1st order, 2-state Kalman Filter for heading estimation
 */
 
float kfGetX(const int i);
void headingKalmanInit(const float x0);
float headingKalman(const float dt, const float Hgps, const bool gps, const float dHgyro, const bool gyro);

#endif
