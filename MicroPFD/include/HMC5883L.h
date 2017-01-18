#ifndef HMC5883L_H
#define HMC5883L_H

#include <string>
#include <exception>
#include <inttypes.h>
#include "sensor.h"


#define HMC5883L_ADDRESS            (0x3C >> 1) // this device only has one address
#define HMC5883L_DEFAULT_ADDRESS    0x1E

#define HMC5883L_RA_CONFIG_A        0x00
#define HMC5883L_RA_CONFIG_B        0x01
#define HMC5883L_RA_MODE            0x02
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAX_L         0x04
#define HMC5883L_RA_DATAZ_H         0x05
#define HMC5883L_RA_DATAZ_L         0x06
#define HMC5883L_RA_DATAY_H         0x07
#define HMC5883L_RA_DATAY_L         0x08
#define HMC5883L_RA_STATUS          0x09
#define HMC5883L_RA_ID_A            0x0A
#define HMC5883L_RA_ID_B            0x0B
#define HMC5883L_RA_ID_C            0x0C

#define HMC5883L_CRA_AVERAGE_BIT    6
#define HMC5883L_CRA_AVERAGE_LENGTH 2
#define HMC5883L_CRA_RATE_BIT       4
#define HMC5883L_CRA_RATE_LENGTH    3
#define HMC5883L_CRA_BIAS_BIT       1
#define HMC5883L_CRA_BIAS_LENGTH    2

#define HMC5883L_AVERAGING_1        0x00
#define HMC5883L_AVERAGING_2        0x01
#define HMC5883L_AVERAGING_4        0x02
#define HMC5883L_AVERAGING_8        0x03

#define HMC5883L_RATE_0P75          0x00
#define HMC5883L_RATE_1P5           0x01
#define HMC5883L_RATE_3             0x02
#define HMC5883L_RATE_7P5           0x03
#define HMC5883L_RATE_15            0x04
#define HMC5883L_RATE_30            0x05
#define HMC5883L_RATE_75            0x06

#define HMC5883L_BIAS_NORMAL        0x00
#define HMC5883L_BIAS_POSITIVE      0x01
#define HMC5883L_BIAS_NEGATIVE      0x02

#define HMC5883L_CRB_GAIN_BIT       7
#define HMC5883L_CRB_GAIN_LENGTH    3

#define HMC5883L_GAIN_1370          0x00
#define HMC5883L_GAIN_1090          0x01
#define HMC5883L_GAIN_820           0x02
#define HMC5883L_GAIN_660           0x03
#define HMC5883L_GAIN_440           0x04
#define HMC5883L_GAIN_390           0x05
#define HMC5883L_GAIN_330           0x06
#define HMC5883L_GAIN_220           0x07

#define HMC5883L_MODEREG_BIT        1
#define HMC5883L_MODEREG_LENGTH     2

#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01
#define HMC5883L_MODE_IDLE          0x02

#define HMC5883L_STATUS_LOCK_BIT    1
#define HMC5883L_STATUS_READY_BIT   0

/*=========================================================================
    RAW MAGNETOMETER DATA TYPE
    -----------------------------------------------------------------------*/
#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H	
    typedef struct magRawData_s
    {
        float x;
        float y;
        float z;
    } Vector;
#endif
/*=========================================================================*/



class HMC5883L
{
    public:
        bool ok;
        std::string err;

        HMC5883L(
            std::string = "/dev/i2c-1",
			int = HMC5883L_ADDRESS
		);
        HMC5883L(uint8_t address);
        virtual ~HMC5883L();

        // CONFIG_A register
        uint8_t 	getSampleAveraging();
        void 		setSampleAveraging(uint8_t averaging);
        uint8_t 	getDataRate();
        void 		setDataRate(uint8_t rate);
        uint8_t 	getMeasurementBias();
        void 		setMeasurementBias(uint8_t t_bias);


        // CONFIG_B register
        uint8_t 	getGain();
        void 		setGain(uint8_t gain);

        // MODE register
        uint8_t 	getMode();
        void 		setMode(uint8_t mode);

        // DATA* registers
		void setOffset(int xo, int yo);
        Vector 		readRaw ();
        float 		readRawX();
        float 		readRawY();
        float 		readRawZ();
		float 		getHeading(float dmg = 0);
		
		Vector 		readNormalize();

        bool 		getEvent(sensors_vec_t *event);


        class smbusIOException : public std::exception {
			public:
				const char* what () throw ();
				const int err;
				smbusIOException (const char*, int);
			private:
				const char *msg;
		};

    protected:
        int 			fd;
        Vector   		r; /* Raw values from last sensor read */
        Vector   		n; /* Raw values from last sensor read */
        uint8_t 		mode;
        uint8_t 		bias;
        uint8_t 		data_rate;
        uint8_t 		samples;
		
		float 			mgPerDigit;
		int 			xOffset, yOffset;

        uint8_t 		buffer[6];
        void    		writeRegister(uint8_t reg, uint8_t value);
        int     		selRegister(uint8_t reg);
        uint8_t 		readRegister(uint8_t reg);

        //int16_t readWord (int addr);
        //int8_t  readByte (int addr);
        //int8_t  readByte () ;
        void 			getREG_A();
        void 			setREG_A();
    private:
};

#endif // HMC5883L_H
