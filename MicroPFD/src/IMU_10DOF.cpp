#include "IMU_10DOF.h"
#include <fcntl.h>
#include <cstring>
#include <cstdio>
#include <cerrno>
#include <cmath>
#include <unistd.h>

using namespace std;

IMU_10DOF::IMU_10DOF()
{
    //ctor
	deltat = 0.0f;
}

IMU_10DOF::~IMU_10DOF()
{
    //dtor
}



void IMU_10DOF::toEulerianAngle(const Quarteniond& q, float& roll, float& pitch, float& yaw)
{
	float ysqr = q.y * q.y;

	// roll (x-axis rotation)
	float t0 = +2.0f * (q.w * q.x + q.y * q.z);
	float t1 = +1.0f - 2.0f * (q.x * q.x + ysqr);
	roll = atan2(t0, t1);

	// pitch (y-axis rotation)
	float t2 = +2.0f * (q.w * q.y - q.z * q.x);
	t2 = t2 > 1.0f ? 1.0f : t2;
	t2 = t2 < -1.0f ? -1.0f : t2;
	pitch = asin(t2);

	// yaw (z-axis rotation)
	float t3 = +2.0f * (q.w * q.z + q.x *q.y);
	float t4 = +1.0f - 2.0f * (ysqr + q.z * q.z);
	yaw = atan2(t3, t4);

	pitch *= 180.0f / M_PI;
    yaw   *= 180.0f / M_PI;
    yaw   -= -18.25; // Declination at Fortaleza-Ceara is 16 degrees and 15
    roll  *= 180.0f / M_PI;
}

// Tilt compensation
bool IMU_10DOF::tiltCompensate(sensors_event_t *imu)
{
  // Pitch & Roll 
  float pitch = atan2(imu->accel.x,(sqrt(imu->accel.y * imu->accel.y + imu->accel.z * imu->accel.z)));
  float roll  = atan2(imu->accel.y,(sqrt(imu->accel.x * imu->accel.x + imu->accel.z * imu->accel.z)));
  
  //if (roll > 0.78 || roll < -0.78 || pitch > 0.78 || pitch < -0.78)
  //{
  //  return false;
  //}

  // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
  float cosRoll = cos(roll);
  float sinRoll = sin(roll);  
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);

  // Tilt compensation
  float Xh = imu->compass.x * cosPitch + imu->compass.z * sinPitch;
  float Yh = imu->compass.x * sinRoll * sinPitch + imu->compass.y * cosRoll - imu->compass.z * sinRoll * cosPitch;
  
  imu->orientation.pitch = pitch * 180.0 / M_PI;
  imu->orientation.roll = roll * 180.0 / M_PI; 
  imu->orientation.yaw = atan2(Yh, Xh) * 180.0 / M_PI;

  return true;
 
}
// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
//void IMU_10DOF::MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
void IMU_10DOF::MadgwickQuaternionUpdate(sensors_event_t *imu)
{
	static uint32_t lastUpdate = 0;
	struct timespec gettime_now;
	clock_gettime(CLOCK_REALTIME, &gettime_now);
	uint32_t start_time = gettime_now.tv_nsec;
	deltat = ((start_time - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
	lastUpdate = start_time;

    //float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
	float q1 = imu->q.w;
	float q2 = imu->q.x;
	float q3 = imu->q.y;
	float q4 = imu->q.z;   // short name local variable for readability

    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

	float ax,ay,az,mx,my,mz,gx,gy,gz;

	ax = imu->accel.x;
	ay = imu->accel.y;
	az = imu->accel.z;

	mx = imu->compass.x;
	my = imu->compass.y;
	mz = imu->compass.z;

	gx = imu->gyro.x * M_PI/180.f;
	gy = imu->gyro.y * M_PI/180.f;
	gz = imu->gyro.z * M_PI/180.f;

    // Normalise accelerometer measurement

    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    imu->q.w = q1 * norm;
    imu->q.x = q2 * norm;
    imu->q.y = q3 * norm;
    imu->q.z = q4 * norm;

    float q[4];
    q[0] = imu->q.w;
    q[1] = imu->q.x;
    q[2] = imu->q.y;
    q[3] = imu->q.z;

    //Quarteniond qt;
    //qt.w = imu->q.w;
    //qt.x = imu->q.x;
    //qt.y = imu->q.y;
    //qt.z = imu->q.z;

    //float pitch, roll, yaw;


    imu->orientation.yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    imu->orientation.pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    imu->orientation.roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    imu->orientation.pitch *= 180.0f / M_PI;
    imu->orientation.yaw   *= 180.0f / M_PI;
    imu->orientation.yaw   -= -18.25; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    imu->orientation.roll  *= 180.0f / M_PI;

    //toEulerianAngle(imu->q, imu->orientation.roll, imu->orientation.pitch, imu->orientation.yaw);
}
