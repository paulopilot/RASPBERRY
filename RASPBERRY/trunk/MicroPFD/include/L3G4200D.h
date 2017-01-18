#ifndef L3G4200D_H
#define L3G4200D_H

#include <string>
#include <exception>
#include <inttypes.h>
#include "sensor.h"

/*=========================================================================
    I2C ADDRESS/BITS AND SETTINGS
    -----------------------------------------------------------------------*/
    #define L3GD20_ADDRESS           (0x69)        // 1101011
    #define L3GD20_POLL_TIMEOUT      (100)         // Maximum number of read attempts
    #define L3GD20_ID                (0xD4)
    #define L3GD20H_ID               (0xD7)
    #define GYRO_SENSITIVITY_250DPS  (0.00875F)    // Roughly 22/256 for fixed point match
    #define GYRO_SENSITIVITY_500DPS  (0.0175F)     // Roughly 45/256
    #define GYRO_SENSITIVITY_2000DPS (0.070F)      // Roughly 18/256

    #define GYRO_DATA_VALID          1
    #define GYRO_DATA_INVALID        0              //  32196640 felipe
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    typedef enum
    {                                             // DEFAULT    TYPE
      GYRO_REGISTER_WHO_AM_I            = 0x0F,   // 11010100   r
      GYRO_REGISTER_CTRL_REG1           = 0x20,   // 00000111   rw
      GYRO_REGISTER_CTRL_REG2           = 0x21,   // 00000000   rw
      GYRO_REGISTER_CTRL_REG3           = 0x22,   // 00000000   rw
      GYRO_REGISTER_CTRL_REG4           = 0x23,   // 00000000   rw
      GYRO_REGISTER_CTRL_REG5           = 0x24,   // 00000000   rw
      GYRO_REGISTER_REFERENCE           = 0x25,   // 00000000   rw
      GYRO_REGISTER_OUT_TEMP            = 0x26,   //            r
      GYRO_REGISTER_STATUS_REG          = 0x27,   //            r
      GYRO_REGISTER_OUT_X_L             = 0x28,   //            r
      GYRO_REGISTER_OUT_X_H             = 0x29,   //            r
      GYRO_REGISTER_OUT_Y_L             = 0x2A,   //            r
      GYRO_REGISTER_OUT_Y_H             = 0x2B,   //            r
      GYRO_REGISTER_OUT_Z_L             = 0x2C,   //            r
      GYRO_REGISTER_OUT_Z_H             = 0x2D,   //            r
      GYRO_REGISTER_FIFO_CTRL_REG       = 0x2E,   // 00000000   rw
      GYRO_REGISTER_FIFO_SRC_REG        = 0x2F,   //            r
      GYRO_REGISTER_INT1_CFG            = 0x30,   // 00000000   rw
      GYRO_REGISTER_INT1_SRC            = 0x31,   //            r
      GYRO_REGISTER_TSH_XH              = 0x32,   // 00000000   rw
      GYRO_REGISTER_TSH_XL              = 0x33,   // 00000000   rw
      GYRO_REGISTER_TSH_YH              = 0x34,   // 00000000   rw
      GYRO_REGISTER_TSH_YL              = 0x35,   // 00000000   rw
      GYRO_REGISTER_TSH_ZH              = 0x36,   // 00000000   rw
      GYRO_REGISTER_TSH_ZL              = 0x37,   // 00000000   rw
      GYRO_REGISTER_INT1_DURATION       = 0x38    // 00000000   rw
    } gyroRegisters_t;
/*=========================================================================*/

/*=========================================================================
    OPTIONAL SPEED SETTINGS
    -----------------------------------------------------------------------*/
    typedef enum
    {
      GYRO_RANGE_250DPS  = 250,
      GYRO_RANGE_500DPS  = 500,
      GYRO_RANGE_2000DPS = 2000
    } gyroRange_t;
/*=========================================================================*/

/*=========================================================================
    RAW GYROSCOPE DATA TYPE
    -----------------------------------------------------------------------*/
#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H		
    typedef struct gyroRawData_s
    {
        float x;
        float y;
        float z;
    } Vector;
#endif
/*=========================================================================*/

class L3G4200D
{
    public:
        bool ok;
        std::string err;

        L3G4200D(
            std::string = "/dev/i2c-1",
			int = L3GD20_ADDRESS
		);
        virtual ~L3G4200D();

        Vector readRaw();
        Vector readNormalize();
        uint8_t getThreshold(void);
        void setThreshold(uint8_t multiple = 1);
        void calibrate(uint8_t samples = 50);

        bool begin(gyroRange_t rng = GYRO_RANGE_2000DPS) ;
		uint8_t readTemperature(void);
        bool getEvent(sensors_vec_t* event);

        class smbusIOException : public std::exception {
			public:
				const char* what () throw ();
				const int err;
				smbusIOException (const char*, int);
			private:
				const char *msg;
		};
    protected:
        int fd;
        gyroRange_t     _range;
        int32_t         _sensorID;
        bool            _autoRangeEnabled;

        Vector   		r; /* Raw values from last sensor read */
        Vector   		n; /* Raw values from last sensor read */
        Vector   		d; /* Raw values from last sensor read */
        Vector   		t; /* Raw values from last sensor read */

        uint8_t         buffer[6];
        bool            useCalibrate;
        float           actualThreshold;
        float           dpsPerDigit;
        float           thresholdX;
        float           thresholdY;
        float           thresholdZ;



        void    writeRegister(uint8_t reg, uint8_t value);
        int     selRegister(uint8_t reg);
        uint8_t readRegister(uint8_t reg);
        void delayus (int delay_us);
    private:
};

#endif // L3G4200D_H
