/*
* Copyright (C) 2008 The Android Open Source Project
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software< /span>
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#ifndef SENSOR_H_INCLUDED
#define SENSOR_H_INCLUDED

#include <inttypes.h>

/* Constants */
#define SENSORS_GRAVITY_EARTH             (9.80665F)              /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_MOON              (1.6F)                  /**< The moon's gravity in m/s^2 */
#define SENSORS_GRAVITY_SUN               (275.0F)                /**< The sun's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD          (SENSORS_GRAVITY_EARTH)
#define SENSORS_MAGFIELD_EARTH_MAX        (60.0F)                 /**< Maximum magnetic field on Earth's surface */
#define SENSORS_MAGFIELD_EARTH_MIN        (30.0F)                 /**< Minimum magnetic field on Earth's surface */
#define SENSORS_PRESSURE_SEALEVELHPA      (1013.25F)              /**< Average sea level pressure is 1013.25 hPa */
#define SENSORS_DPS_TO_RADS               (0.017453293F)          /**< Degrees/s to rad/s multiplier */
#define SENSORS_GAUSS_TO_MICROTESLA       (100)

/******************************************************************
HMC5883L    (3-Axis Digital Compass),       I2C Address 0x1E
ADXL345     (3-Axis Digital Accelerometer), I2C Address 0×53
L3G4200D    (3-Axis Angular Rate Sensor),   I2C Address 0×69
BMP085      (Baro Press / Temp Sensor),     I2C Address 0×77
*******************************************************************/

/** Sensor types */
typedef enum
{
  SENSOR_TYPE_ACCELEROMETER         = (1),   /**< Gravity + linear acceleration */
  SENSOR_TYPE_MAGNETIC_FIELD        = (2),
  SENSOR_TYPE_ORIENTATION           = (3),
  SENSOR_TYPE_GYROSCOPE             = (4),
  SENSOR_TYPE_LIGHT                 = (5),
  SENSOR_TYPE_PRESSURE              = (6),
  SENSOR_TYPE_PROXIMITY             = (8),
  SENSOR_TYPE_GRAVITY               = (9),
  SENSOR_TYPE_LINEAR_ACCELERATION   = (10),  /**< Acceleration not including gravity */
  SENSOR_TYPE_ROTATION_VECTOR       = (11),
  SENSOR_TYPE_RELATIVE_HUMIDITY     = (12),
  SENSOR_TYPE_AMBIENT_TEMPERATURE   = (13),
  SENSOR_TYPE_VOLTAGE               = (15),
  SENSOR_TYPE_CURRENT               = (16),
  SENSOR_TYPE_COLOR                 = (17)
} sensors_type_t;

/** struct sensors_vec_s is used to return a vector in a common format. */

typedef struct {
    union {
		int32_t sensor_id;                        /**< unique sensor identifier */
		int32_t  type;
        struct {
            float x;
            float y;
            float z;
        };

        struct {
            float temperature;
            float altitude;
			float pressure;
        };
    };
    int8_t status;
} sensors_vec_t;


typedef struct {
    float yaw;
    float pitch;
    float roll;
    float heading;
} orientation_t;

/* Quartenion struct */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quarteniond;

/* Sensor event */
typedef struct
{
    int32_t version;							/**< must be sizeof(struct sensors_event_t) */
    int32_t timestamp;							/**< time is in milliseconds */
    //union
    //{
        Quarteniond     q;						/**< vector to hold quaternion */
        sensors_vec_t   accel;         			/**< acceleration values are in meter per second per second (m/s^2) */
        sensors_vec_t   compass;				/**< magnetic vector values are in micro-Tesla (uT) */
		sensors_vec_t   gyro;					/**< gyroscope values are in rad/s */
		sensors_vec_t	baro;
        orientation_t   orientation;			/**< orientation values are in degrees */

        //float           temperature;          /**< temperature is in degrees centigrade (Celsius) */
        float           distance;				/**< distance in centimeters */
        float           light;					/**< light in SI lux units */
        //float           pressure;             /**< pressure in hectopascal (hPa) */
        float           current;				/**< current in milliamps (mA) */
        float           voltage;				/**< voltage in volts (V) */
        //float           altitude;
    //};
} sensors_event_t;


/* Sensor details (40 bytes) */
/** struct sensor_s is used to describe basic information about a specific sensor. */
typedef struct
{
    char     name[12];                        /**< sensor name */
    int32_t  version;                         /**< version of the hardware + driver */
    int32_t  sensor_id;                       /**< unique sensor identifier */
    int32_t  type;                            /**< this sensor's type (ex. SENSOR_TYPE_LIGHT) */
    float    max_value;                       /**< maximum value of this sensor's value in SI units */
    float    min_value;                       /**< minimum value of this sensor's value in SI units */
    float    resolution;                      /**< smallest difference between two values reported by this sensor */
    int32_t  min_delay;                       /**< min delay in microseconds between events. zero = not a constant rate */
} sensor_t;

class Sensor {
 public:
  // Constructor(s)
  Sensor() {}
  virtual ~Sensor() {}

  // These must be defined by the subclass
  virtual void enableAutoRange(bool enabled) {};
  //virtual bool getEvent(sensors_event_t*) = 0;
  virtual bool getEvent(sensors_vec_t*) = 0;
  virtual void getSensor(sensor_t*) = 0;

 private:
  bool _autoRange;
};

#endif // SENSOR_H_INCLUDED
