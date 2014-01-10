#include "mbed.h"
#include "Matrix.h"
#include "Fixed.h"
#include "FixedMatrix.h"
#include "util.h"

#define DEBUG 1

//#define clamp360(x) ((((x) < 0) ? 360: 0) + fmod((x), 360))

/*
 * Kalman Filter Setup
 */

#if 1
static FixedMatrix x(2,1);
static FixedMatrix z(2,1);
static FixedMatrix A(2,2);
static FixedMatrix H(2,2);
static FixedMatrix K(2,2);
static FixedMatrix P(2,2);
static FixedMatrix R(2,2);
static FixedMatrix Q(2,2);
static FixedMatrix I(2,2);
#else
static float x[2]={ 0, 0 };                 // System State: hdg, hdg rate
float z[2]={ 0, 0 };                        // measurements, hdg, hdg rate
static float A[4]={ 1, 0, 0, 1};            // State transition matrix; A[1] should be dt
static float H[4]={ 1, 0, 0, 1 };           // Observer matrix maps measurements to state transition
Fixed K[4]={ 0, 0, 0, 0 };                  // Kalman gain
static float P[4]={ 1000, 0, 0, 1000 };     // Covariance matrix
static float R[4]={ 100, 0, 0, 0.03 };        // Measurement noise, hdg, hdg rate
static float Q[4]={ 100, 0, 0, 0.01 };        // Process noise matrix
static float I[4]={ 1, 0, 0, 1 };           // Identity matrix
#endif

float kfGetX(int i)
{
    return (i >= 0 && i < 2) ? x[i].toFloat() : float(0);
}

/** headingKalmanInit
 *
 * initialize x, z, K, and P
 */
void headingKalmanInit(float x0)
{
#if 1
	A(0,0) = 1;	A(1,1) = 1;
	H(0,0) = 1;	H(1,1) = 1;
	P(0,0) = 1000; P(1,1) = 1000;
	R(0,0) = 100; R(1,1) = 0.03;
	Q(0,0) = 100; Q(1,1) = 0.01;
	I(0,0) = 1; I(1,1) = 1;
	x(0,0) = x0;
#else
	x[0] = x0;
#endif
}


/* headingKalman 
 *
 * Implements a 1-dimensional, 1st order Kalman Filter
 *
 * That is, it deals with heading and heading rate (h and h') but no other
 * state variables.  The state equations are:
 *
 *                     X    =    A       X^
 * h = h + h'dt -->  | h  | = | 1 dt | | h  |
 * h' = h'           | h' |   | 0  1 | | h' |
 *
 * Kalman Filtering is not that hard. If it's hard you haven't found the right
 * teacher. Try taking CS373 from Udacity.com
 *
 * This notation is Octave (Matlab) syntax and is based on the Bishop-Welch
 * paper and references the equation numbers in that paper.
 * http://www.cs.unc.edu/~welch/kalman/kalmanIntro.html
 *
 * returns : current heading estimate
 */
float headingKalman(float dt, float Hgps, bool gps, float dHgyro, bool gyro)
{
    A[1] = dt;

    /* Initialize, first time thru
    x = H*z0
    */

    //fprintf(stdout, "gyro? %c  gps? %c\n", (gyro)?'Y':'N', (gps)?'Y':'N');
            
    // Depending on what sensor measurements we've gotten,
    // switch between observer (H) matrices and measurement noise (R) matrices
    // TODO 3 incorporate HDOP or sat count in R
    if (gps) {
        H[0] = 1.0;
        z[0] = Hgps;
    } else {
        H[0] = 0;
        z[0] = 0;
    }

    if (gyro) {
        H[3] = 1.0;
        z[1] = dHgyro;
    } else {
        H[3] = 0;
        z[1] = 0;
    }

    //Matrix_print(2,2, A, "1. A");
    //Matrix_print(2,2, P, "   P");
    //Matrix_print(2,1, x, "   x");
    //Matrix_print(2,1, K, "   K");
    //Matrix_print(2,2, H, "2. H");
    //Matrix_print(2,1, z, "   z");
   
   /**********************************************************************
     * Predict
     %
     * In this step we "move" our state estimate according to the equation
     *
     * x = A*x; // Eq 1.9
     ***********************************************************************/
    x = A*x;
    
    //Matrix_print(2,1, xp, "3. xp");

    /**********************************************************************
     * We also have to "move" our uncertainty and add noise. Whenever we move,
     * we lose certainty because of system noise.
     *
     * P = A*P*A' + Q; // Eq 1.10
     ***********************************************************************/
    P = A*P;
    P = P*~A;
    P += Q;

    //Matrix_print(2,2, P, "4. P");

    /**********************************************************************
     * Measurement aka Correct
     * First, we have to figure out the Kalman Gain which is basically how
     * much we trust the sensor measurement versus our prediction.
     *
     * K = P*H'*inv(H*P*H' + R);    // Eq 1.11
     ***********************************************************************/
    K = P*~H*!(H*P*~H + R);

    //Matrix_print(2,2, K,    "5. K");
        
    /**********************************************************************
     * Then we determine the discrepancy between prediction and measurement 
     * with the "Innovation" or Residual: z-H*x, multiply that by the 
     * Kalman gain to correct the estimate towards the prediction a little 
     * at a time.
     *
     * x = x + K*(z-H*x);            // Eq 1.12
     ***********************************************************************/
    x += K*(z-H*x);
    
    x[0] = clamp360(x[0]);    // Clamp to 0-360 range

    //Matrix_print(2,1, x, "6. x");

    /**********************************************************************
     * We also have to adjust the certainty. With a new measurement, the 
     * estimate certainty always increases.
     *
     * P = (I-K*H)*P;                // Eq 1.13
     ***********************************************************************/
    P = (I-K*H)*P;

    //Matrix_print(2,2, P, "7. P");
    return x[0].toFloat();
}
