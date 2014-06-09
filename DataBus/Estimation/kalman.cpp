#include "mbed.h"
#include "Matrix.h"
#include "util.h"

#define DEBUG 1

//#define clamp360(x) ((((x) < 0) ? 360: 0) + fmod((x), 360))

/*
 * Kalman Filter Setup
 */
static float x[3]={ 0,
					0,
					0 };                 // System State: hdg, hdg rate, bias

static float A[9]={ 1, 0, 0,
				    0, 1, 0,
				    0, 0, 1};            // State transition matrix; A[1]=dt, A[2]=-dt

static float K[6]={ 0, 0,
					0, 0,
					0, 0 };                  // Kalman gain

static float P[9]={ 1000, 0, 0,
					0, 1000, 0,
					0, 0, 1000 };     // Covariance matrix

static float R[4]={ 0.25, 0,
					0, 0.03 };        // Measurement noise, hdg, hdg rate

static float Q[9]={ 1, 0, 0,
					0, 0.1, 0,
					0, 0, 0.00005 };        // Process noise matrix

static float I[9]={ 1, 0, 0,
					0, 1, 0,
					0, 0, 1 };           // Identity matrix


/** headingKalmanInit
 *
 * initialize x, z, K, and P
 */
void headingKalmanInit(float x0)
{
    x[0] = x0;
    x[1] = 0;
    x[2] = 0;
}


/* headingKalman 
 *
 * Implements a 1-dimensional, 1st order Kalman Filter
 *
 * That is, it deals with heading and heading rate (h and h') and bias, but no other
 * state variables.  The state equations are:
 *
 *                            X    =      A         X^
 * ^h = h + h'dt -->  | h  | = | 1 dt  0 | | h  |
 * ^h' = h'           | h' |   | 0  1  0 | | h' |
 * ^bias = bias       | b  |   | 0  0  1 | | b  |
 *
 * The Observation model is set up so that gyro measurement represents
 * both heading rate _and_ bias:
 *
 * H = | 1 0 0 |
 *     | 0 1 1 |
 *
 * Kalman Filtering is not that hard. If it's hard you haven't found the right
 * teacher. Try taking CS373 from Udacity.com
 *
 * Notation based on the Bishop-Welch paper and references the equation numbers in that paper.
 * http://www.cs.unc.edu/~welch/kalman/kalmanIntro.html
 *
 * returns : current heading estimate
 */
void headingKalman(float dt, float Hgps, bool gps, float dHgyro, bool gyro)
{
    float z[2]={ 0,
    		     0 };		// measurements, hdg, hdg rate
    float H[6]={ 0, 0, 0,
    			 0, 0, 0 }; // Observer matrix maps measurements to state transition


    A[1] = dt;

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
        H[4] = 1.0;
        H[5] = 1.0;
        z[1] = dHgyro;
    } else {
        H[4] = 0;
        H[5] = 0;
        z[1] = 0;
    }

//    Matrix_print(3,3, A, "1. A");
//    Matrix_print(3,3, P, "   P");
//    Matrix_print(3,1, x, "   x");
//    Matrix_print(3,2, K, "   K");
//    Matrix_print(2,3, H, "2. H");
//    Matrix_print(2,1, z, "   z");
   
   /**********************************************************************
     * Predict
     %
     * In this step we "move" our state estimate according to the equation
     *
     * x = A*x; // Eq 1.9
     ***********************************************************************/
    float xp[3];
    Matrix_Multiply(3,3,1, xp, A, x);
    
//    Matrix_print(3,1, x, "3. x");
//    Matrix_print(3,1, xp, "3. xp");

    /**********************************************************************
     * We also have to "move" our uncertainty and add noise. Whenever we move,
     * we lose certainty because of system noise.
     *
     * P = A*P*A' + Q; // Eq 1.10
     ***********************************************************************/
    float At[9];
    Matrix_Transpose(3,3, At, A);
    float AP[9];
    Matrix_Multiply(3,3,3, AP, A, P);
    float APAt[9];
    Matrix_Multiply(3,3,3, APAt, AP, At);
    Matrix_Add(3,3, P, APAt, Q);

//    Matrix_print(3,3, P, "4. P");

    /**********************************************************************
     * Measurement aka Correct
     * First, we have to figure out the Kalman Gain which is basically how
     * much we trust the sensor measurement versus our prediction.
     *
     * K = P*H'*inv(H*P*H' + R);    // Eq 1.11
     ***********************************************************************/
    float Ht[6];
//    Matrix_print(2,3, H,    "5. H");
    Matrix_Transpose(2,3, Ht, H);
//    Matrix_print(3,2, Ht,    "5. Ht");

    float HP[6];
//    Matrix_print(3,3, P,    "5. P");
    Matrix_Multiply(2,3,3, HP, H, P);
//    Matrix_print(2,3, HP,    "5. HP");

    float HPHt[4];
    Matrix_Multiply(2,3,2, HPHt, HP, Ht);
//    Matrix_print(2,2, HPHt,    "5. HPHt");
    
    float HPHtR[4];
//    Matrix_print(2,2, R,    "5. R");
    Matrix_Add(2,2, HPHtR, HPHt, R);
//    Matrix_print(2,2, HPHtR,    "5. HPHtR");

    Matrix_Inverse(2, HPHtR);
//    Matrix_print(2,2, HPHtR,    "5. HPHtR");

    float PHt[6];
//    Matrix_print(3,3, P,    "5. P");
//    Matrix_print(3,2, Ht,    "5. Ht");
    Matrix_Multiply(3,3,2, PHt, P, Ht);
//    Matrix_print(3,2, PHt,    "5. PHt");
    
    Matrix_Multiply(3,2,2, K, PHt, HPHtR);
    
//    Matrix_print(3,2, K,    "5. K");
        
    /**********************************************************************
     * Then we determine the discrepancy between prediction and measurement 
     * with the "Innovation" or Residual: z-H*x, multiply that by the 
     * Kalman gain to correct the estimate towards the prediction a little 
     * at a time.
     *
     * x = x + K*(z-H*x);            // Eq 1.12
     ***********************************************************************/
    float Hx[2];
    Matrix_Multiply(2,3,1, Hx, H, xp);
    
//    Matrix_print(2,3, H, "6. H");
//    Matrix_print(3,1, xp, "6. xp");
//    Matrix_print(2,1, Hx, "6. Hx");
    
    float zHx[2];
    Matrix_Subtract(2,1, zHx, z, Hx);
    zHx[0] = clamp180(zHx[0]);

//    Matrix_print(2,1, z, "6. z");
//    Matrix_print(2,1, zHx, "6. zHx");
    
    float KzHx[3];
    Matrix_Multiply(3,2,1, KzHx, K, zHx);

//    Matrix_print(2,2, K, "6. K");
//    Matrix_print(3,1, xp, "6. xp");
//    Matrix_print(2,1, KzHx, "6. KzHx");
    
    Matrix_Add(3,1, x, xp, KzHx);
    x[0] = clamp360(x[0]);    // Clamp to 0-360 range

//    Matrix_print(3,1, x, "6. x");

    /**********************************************************************
     * We also have to adjust the certainty. With a new measurement, the 
     * estimate certainty always increases.
     *
     * P = (I-K*H)*P;                // Eq 1.13
     ***********************************************************************/
    float KH[9];
//    Matrix_print(3,2, K, "7. K");
    Matrix_Multiply(3,2,3, KH, K, H);
//    Matrix_print(3,3, KH, "7. KH");
    float IKH[9];
    Matrix_Subtract(3,3, IKH, I, KH);
//    Matrix_print(3,3, IKH, "7. IKH");
    float P2[9];
    Matrix_Multiply(3,3,3, P2, IKH, P);
    Matrix_Copy(3,3, P, P2);

//    Matrix_print(3,3, P, "7. P");
}


float kfGetX(int i)
{
    return (i >= 0 && i < 3) ? x[i] : 0xFFFFFFFF;
}
