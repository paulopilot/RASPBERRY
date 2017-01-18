#ifndef IMU_10DOF_H
#define IMU_10DOF_H


#include "sensor.h"
#include "ADXL345.h"
#include "BMP085.h"
#include <math.h>
#include <time.h>
#include <stdio.h>

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError M_PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 40 deg/s)
#define GyroMeasDrift M_PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

/** Sensor axis */
typedef enum
{
  SENSOR_AXIS_X  = (1),
  SENSOR_AXIS_Y  = (2),
  SENSOR_AXIS_Z  = (3)
} sensors_axis_t;

class IMU_10DOF
{
    public:
        IMU_10DOF();
        virtual ~IMU_10DOF();
        //bool accelGetOrientation(sensors_event_t *event, sensors_vec_t *orientation);
        //bool magTiltCompensation(sensors_axis_t axis, sensors_event_t *mag_event, sensors_event_t *accel_event);
        //bool magGetOrientation(sensors_axis_t axis, sensors_event_t *event, sensors_vec_t *orientation);
        //bool fusionGetOrientation(sensors_event_t *accel_event, sensors_event_t *mag_event, sensors_vec_t *orientation);

        //bool getOrientation(ADXL345::reading *accel, sensors_vec_t *orientation);
		bool tiltCompensate(sensors_event_t *imu);
        void toEulerianAngle(const Quarteniond& q, float& roll, float& pitch, float& yaw);
		void MadgwickQuaternionUpdate(sensors_event_t *imu);
    protected:
		float	deltat;
    private:
};

#endif // IMU_10DOF_H
