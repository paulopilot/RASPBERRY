#include "ADXL345.h"
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <cstring>
#include <cstdio>
#include <cerrno>
#include <cmath>
#include <unistd.h>


using namespace std;

/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register
*/
/**************************************************************************/
void ADXL345::writeRegister(uint8_t reg, uint8_t value) {


    if(i2c_smbus_write_byte_data(fd, reg, value) < 0)
    {
        char str[32];
		sprintf(str, "readWord() 0x%x", reg);
		throw ADXL345::smbusIOException (
			str,
			errno
		);
    }
}

int ADXL345::selRegister(uint8_t reg) {

    buffer[0] = reg;

    int result = write(fd, buffer, 1);

    if (result != 1)
    {
        char str[32];
        sprintf(str, "writeToDevice() 0x%x", fd);
        throw ADXL345::smbusIOException (
            str,
            errno
        );
    }

    return result;
}

/**************************************************************************/
/*!
    @brief  Reads 8-bits from the specified register
*/
/**************************************************************************/
uint8_t ADXL345::readRegister(uint8_t reg) {
    uint8_t ret_val;

    ret_val = i2c_smbus_read_byte_data(fd, reg);

    if (ret_val < 0) {
        char str[32];
		sprintf(str, "readWord() 0x%x", reg);
		throw ADXL345::smbusIOException (
			str,
			errno
		);
    }

    return ret_val;
}

/**************************************************************************/
/*!
    @brief  Reads 16-bits from the specified register
*/
/**************************************************************************/
int16_t ADXL345::readWord (int addr) {
	int word = i2c_smbus_read_word_data(fd, addr);
	if (word == -1) {
		char str[32];
		sprintf(str, "readWord() 0x%x", addr);
		throw ADXL345::smbusIOException (
			str,
			errno
		);
	}
// The sensor is big-endian!
	return (int16_t)((word >> 8) + ((word & 0xff) << 8));
}

/**************************************************************************/
/*!
    @brief  Get ID device
*/
/**************************************************************************/
uint8_t ADXL345::getDeviceID(void) {
  // Check device ID register
  return readRegister(ADXL345_REG_DEVID);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent X axis value
*/
/**************************************************************************/
int16_t ADXL345::getX(void) {
  return readWord(ADXL345_REG_DATAX0);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent Y axis value
*/
/**************************************************************************/
int16_t ADXL345::getY(void) {
    return readWord(ADXL345_REG_DATAY0);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent Z axis value
*/
/**************************************************************************/
int16_t ADXL345::getZ(void) {
  return readWord(ADXL345_REG_DATAZ0);
}

void ADXL345::getXYZ(int16_t *x, int16_t *y, int16_t *z) {

    //selRegister(ADXL345_REG_DATAX0);

    //if (read(fd, buffer, 6) != 6) {


    //}else{
    i2c_smbus_read_i2c_block_data(fd, ADXL345_REG_DATAX0, 6, &buffer[0]);
        *x = (((int16_t)buffer[1]) << 8) | buffer[0];
        *y = (((int16_t)buffer[3]) << 8) | buffer[2];
        *z = (((int16_t)buffer[5]) << 8) | buffer[4];

        //printf("X=0x%02x%02x", buffer[1], buffer[0] );
        //printf("  Y=0x%02x%02x", buffer[3], buffer[2] );
        //printf("  Z=0x%02x%02x\n", buffer[5], buffer[4] );

        //printf("RAW REGISTER X: %+4d  Y: %+4d  Z: %+4d\n", *x, *y, *z);

        //#define ADXL345_X_GAIN 260//446 // GAIN factors
        //#define ADXL345_Y_GAIN 308//300
        //#define ADXL345_Z_GAIN 261//499

        //float accX, accY, accZ;
        //accX = (float)*x;
        //accY = (float)*y;
        //accZ = (float)*z;

        // apply the slope and offset factors
        //accX = ( accX - 194 ) / ADXL345_X_GAIN;
        //accY = ( accY -  41 ) / ADXL345_Y_GAIN;
        //accZ = ( accZ - 499 ) / ADXL345_Z_GAIN;

        //printf("ACCELERATION X: %+0.3fg Y: %+0.3fg  Z: %+0.3fg\n", accX, accY, accZ);



        //printf("X=0x%04x\n",*x );

        //if (*x & (1 << 16 -1)) *x = *x - (1<<16);
        //if (*y & (1 << 16 -1)) *y = *y - (1<<16);
        //if (*z & (1 << 16 -1)) *z = *z - (1<<16);

    //}


 }

ADXL345::reading ADXL345::getXYZ () {

    int16_t x,y,z;
    getXYZ(&x,&y, &z);

	ADXL345::reading data = {
		x,
		y,
		z
	};
	return data;
}


bool ADXL345::getEvent(sensors_vec_t *event) {
    memset(event, 0, sizeof(sensors_vec_t));

    //event->version = sizeof(sensors_vec_t);
    event->sensor_id = _sensorID;
    event->type = SENSOR_TYPE_ACCELEROMETER;

    int16_t x,y,z;
    getXYZ(&x,&y, &z);

    //float accX, accY, accZ;

    //accX = (float)x * ADXL345_MG2G_MULTIPLIER;
    //accY = (float)y * ADXL345_MG2G_MULTIPLIER;
    //accZ = (float)z * ADXL345_MG2G_MULTIPLIER;

    //accX = (accX - (-0.01))/1.05;
    //accY = (accY - (+0.04))/1.07;
    //accZ = (accZ - (+0.02))/1.02;

    //event->x = accX;
	//event->y = accY;
	//event->z = accZ;

	event->x = (float)x * ADXL345_MG2G_MULTIPLIER;// * SENSORS_GRAVITY_STANDARD;
	event->y = (float)y * ADXL345_MG2G_MULTIPLIER;// * SENSORS_GRAVITY_STANDARD;
	event->z = (float)z * ADXL345_MG2G_MULTIPLIER;// * SENSORS_GRAVITY_STANDARD;

    return true;
}

/**************************************************************************/
/*!
    @brief  Reads 16-bits from the specified register
*/
/**************************************************************************/
ADXL345::ADXL345(string dev, int addr) :
    ok(true),
	err("")

{

    _range = ADXL345_RANGE_2_G;
	_sensorID = 10085;

    fd = open(dev.c_str(), O_RDWR);
    if (fd < 0) {
		ok = false;
		err = "open() fail: ";
		err += strerror(errno);
		return;
	}
	if (ioctl(fd, I2C_SLAVE, addr) < 0) {
		ok = false;
		err = "ioctl() fail: ";
		err += strerror(errno);
		return;
	}

	/* Check connection */
    uint8_t deviceid = getDeviceID();

    if (deviceid != 0xE5)
    {
        /* No ADXL345 detected ... return false */
        ok = false;
		err = "No ADXL345 detected: ";
		err += strerror(errno);
        return;
    }

	//offsetCalibration();

	writeRegister(ADXL345_REG_POWER_CTL, 0x00); // Put device in standby mode and clear sleep bit 2
	usleep(100*1000);  // Let device settle down
	writeRegister(ADXL345_REG_POWER_CTL, 0x08); // Put device in normal mode
	// Set accelerometer configuration; interrupt active high, left justify MSB
	//writeRegister(ADXL345_REG_DATA_FORMAT, 0x04 | ADXL345_RANGE_2_G); // Set full scale range for the accelerometer
	setRange(ADXL345_RANGE_2_G);

	// Choose ODR and bandwidth
	writeRegister(ADXL345_REG_BW_RATE, ADXL345_DATARATE_100_HZ); // Select normal power operation, and ODR and bandwidth

	//writeRegister(ADXL345_REG_INT_ENABLE, 0x80);  // Enable data ready interrupt
	//writeRegister(ADXL345_REG_INT_MAP, 0x00);     // Enable data ready interrupt on INT_1

	writeRegister(ADXL345_REG_FIFO_CTL, 0x00);    // Bypass FIFO

	writeRegister(ADXL345_REG_POWER_CTL, 0x08); // Put device in normal mode
}

ADXL345::~ADXL345()
{
     close(fd);
}

void ADXL345::offsetCalibration()
{
	uint8_t 		data[6] = {0, 0, 0, 0, 0, 0};
	int 			abias[3] = {0, 0, 0};
	int32_t 		accel_bias[3] = {0, 0, 0};
	uint8_t		samples, ii;


	// wake up device
	writeRegister(ADXL345_REG_POWER_CTL, 0x00); // Put device in standby mode and clear sleep bit 2
	usleep(1100);

		// Set accelerometer configuration; interrupt active high, left justify MSB
	//writeRegister(ADXL345_REG_DATA_FORMAT, 0x04 | 0x00); // Set full scale range to 2g for the bias calculation
	writeRegister(ADXL345_REG_DATA_FORMAT, 0x0B); // Set full scale range to 2g for the bias calculation
	uint16_t  accelsensitivity = 256;  // = 256 LSB/g at 2g full scale++


	writeRegister(ADXL345_REG_POWER_CTL, 0x08); // Put device in normal mode
	usleep(1100);



	// Choose ODR and bandwidth
	writeRegister(ADXL345_REG_BW_RATE, 0x09); // Select normal power operation, and 100 Hz ODR and 50 Hz bandwidth
	usleep(10*1000);

	writeRegister(ADXL345_REG_FIFO_CTL, 0x40 | 0x2F);    // Enable FIFO stream mode | collect 32 FIFO samples
	usleep(1000*1000);

	//samples = readByte(ADXL345_ADDRESS, ADXL345_REG_FIFO_STATUS);
	samples = readRegister(ADXL345_REG_FIFO_STATUS);


	for(ii = 0; ii < samples ; ii++) {
		//readBytes(ADXL345_ADDRESS, ADXL345_REG_DATAX0, 6, &data[0]);
		//read(fd, buffer, 6)
		i2c_smbus_read_i2c_block_data(fd, ADXL345_REG_DATAX0, 6, &data[0]);
		accel_bias[0] += (((int16_t)data[1] << 8) | data[0]) >> 6;
		accel_bias[1] += (((int16_t)data[3] << 8) | data[2]) >> 6;
		accel_bias[2] += (((int16_t)data[5] << 8) | data[4]) >> 6;
	}

	accel_bias[0] /= samples; // average the data
	accel_bias[1] /= samples;
	accel_bias[2] /= samples;

	// Remove gravity from z-axis accelerometer bias value
	if(accel_bias[2] > 0) {
		accel_bias[2] -= accelsensitivity;
	} else {
		accel_bias[2] += accelsensitivity;
	}

	abias[0] = (int)((-accel_bias[0]/4) & 0xFF); // offset register are 8 bit 2s-complement, so have sensitivity 1/4 of 2g full scale
	abias[1] = (int)((-accel_bias[1]/4) & 0xFF);
	abias[2] = (int)((-accel_bias[2]/4) & 0xFF);

	writeRegister(ADXL345_REG_OFSX, abias[0]);
	writeRegister(ADXL345_REG_OFSY, abias[1]);
	writeRegister(ADXL345_REG_OFSZ, abias[2]);

	writeRegister(ADXL345_REG_OFSX, 0); //-6
	writeRegister(ADXL345_REG_OFSY, 0);//+4
	writeRegister(ADXL345_REG_OFSZ, 0);//+3
}



/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
void ADXL345::setRange(range_t range)
{
  /* Red the data format register to preserve bits */
  uint8_t format = readRegister(ADXL345_REG_DATA_FORMAT);

  /* Update the data rate */
  format &= ~0x0F;
  format |= range;

  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;

  /* Write the register back to the IC */
  writeRegister(ADXL345_REG_DATA_FORMAT, format);

  /* Keep track of the current range (to avoid readbacks) */
  _range = range;
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
range_t ADXL345::getRange(void)
{
  /* Red the data format register to preserve bits */
  return (range_t)(readRegister(ADXL345_REG_DATA_FORMAT) & 0x03);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the ADXL345 (controls power consumption)
*/
/**************************************************************************/
void ADXL345::setDataRate(dataRate_t dataRate)
{
  /* Note: The LOW_POWER bits are currently ignored and we always keep
     the device in 'normal' mode */
  writeRegister(ADXL345_REG_BW_RATE, dataRate);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the ADXL345 (controls power consumption)
*/
/**************************************************************************/
dataRate_t ADXL345::getDataRate(void)
{
  return (dataRate_t)(readRegister(ADXL345_REG_BW_RATE) & 0x0F);
}

/* Class smbusIOException */

ADXL345::smbusIOException::smbusIOException (const char* m, int err) :
	err(err), msg(m)
{ }

const char* ADXL345::smbusIOException::what () throw () {
	string m = msg;
	if (err) {
		m += ":\n";
		m += strerror(errno);
	}
	return m.c_str();
}

