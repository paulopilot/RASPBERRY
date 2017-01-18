#ifndef 10DOF_H
#define 10DOF_H

#include "sensor.h"
#include "ADXL345.h"
#include "BMP085.h"


class 10DOF
{
    public:
        typedef struct {
			float pitch;
			float roll;
			float yaw;
			float temp;
			float alt;
		} reading;
        10DOF();
        virtual ~10DOF();

        bool accelGetOrientation(ADXL345::reading *accel, sensors_vec_t *orientation)
    protected:
    private:
};

#endif // 10DOF_H
