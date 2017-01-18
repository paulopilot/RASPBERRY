#include "L3G4200D.h"
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
void L3G4200D::writeRegister(uint8_t reg, uint8_t value) {

    if(i2c_smbus_write_byte_data(fd, reg, value) < 0)
    {
        char str[32];
		sprintf(str, "writeRegister(0x%x) ", reg);
		throw L3G4200D::smbusIOException (
			str,
			errno
		);
    }
}

int L3G4200D::selRegister(uint8_t reg) {

    buffer[0] = reg;

    int result = write(fd, buffer, 1);

    if (result != 1)
    {
        char str[32];
        sprintf(str, "writeToDevice() 0x%x", fd);
        throw L3G4200D::smbusIOException (
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
uint8_t L3G4200D::readRegister(uint8_t reg) {

    uint8_t ret_val;

    ret_val = i2c_smbus_read_byte_data(fd, reg);

    if (ret_val < 0) {
        char str[32];
		sprintf(str, "readRegister() 0x%x", reg);
		throw L3G4200D::smbusIOException (
			str,
			errno
		);
    }

    return ret_val;

}


L3G4200D::L3G4200D(string dev, int addr)  :
    ok(true),
	err("")

{
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
    //ctor
    _autoRangeEnabled = false;

    uint8_t id = readRegister(GYRO_REGISTER_WHO_AM_I);
    //Serial.println(id, HEX);
    if ((id != L3GD20_ID) && (id != L3GD20H_ID))
    {
        ok = false;
		err = "Gyro sensor not found: ";
		err += strerror(errno);
		return;
    }
}

L3G4200D::~L3G4200D()
{
    //dtor
    close(fd);
}

bool L3G4200D::begin(gyroRange_t rng) {

    // Reset calibrate values
    d.x = 0;
    d.y = 0;
    d.z = 0;
    useCalibrate = false;

    // Reset threshold values
    t.x = 0;
    t.y = 0;
    t.z = 0;
    actualThreshold = 0;

    /* Set the range the an appropriate value */
    _range = rng;

    /* Set CTRL_REG1 (0x20)
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
    7-6  DR1/0     Output data rate                                   00
    5-4  BW1/0     Bandwidth selection                                00
      3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
      2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
      1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
      0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */

    /* Reset then switch to normal mode and enable all three channels */
    writeRegister(GYRO_REGISTER_CTRL_REG1, 0x00);
    writeRegister(GYRO_REGISTER_CTRL_REG1, 0x0F);
    /* ------------------------------------------------------------------ */
    /* Set CTRL_REG2 (0x21)
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
    5-4  HPM1/0    High-pass filter mode selection                    00
    3-0  HPCF3..0  High-pass filter cutoff frequency selection      0000 */

    /* Nothing to do ... keep default values */
    writeRegister(GYRO_REGISTER_CTRL_REG2, 0x00);
    /* ------------------------------------------------------------------ */

    /* Set CTRL_REG3 (0x22)
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
      7  I1_Int1   Interrupt enable on INT1 (0=disable,1=enable)       0
      6  I1_Boot   Boot status on INT1 (0=disable,1=enable)            0
      5  H-Lactive Interrupt active config on INT1 (0=high,1=low)      0
      4  PP_OD     Push-Pull/Open-Drain (0=PP, 1=OD)                   0
      3  I2_DRDY   Data ready on DRDY/INT2 (0=disable,1=enable)        0
      2  I2_WTM    FIFO wtrmrk int on DRDY/INT2 (0=dsbl,1=enbl)        0
      1  I2_ORun   FIFO overrun int on DRDY/INT2 (0=dsbl,1=enbl)       0
      0  I2_Empty  FIFI empty int on DRDY/INT2 (0=dsbl,1=enbl)         0 */

    /* Nothing to do ... keep default values */
    /* ------------------------------------------------------------------ */

    /* Set CTRL_REG4 (0x23)
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
      7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
      6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
    5-4  FS1/0     Full scale selection                               00
                                  00 = 250 dps
                                  01 = 500 dps
                                  10 = 2000 dps
                                  11 = 2000 dps
      0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0 */

    /* Adjust resolution if requested */
    switch(_range)
    {
        case GYRO_RANGE_250DPS:
            writeRegister(GYRO_REGISTER_CTRL_REG4, 0x00);
            dpsPerDigit = 0.00875f;
            break;
        case GYRO_RANGE_500DPS:
            writeRegister(GYRO_REGISTER_CTRL_REG4, 0x10);
            dpsPerDigit = 0.0175f;
            break;
        case GYRO_RANGE_2000DPS:
            writeRegister(GYRO_REGISTER_CTRL_REG4, 0x20);
            dpsPerDigit = 0.07f;
            break;
    }

    /* ------------------------------------------------------------------ */

    /* Set CTRL_REG5 (0x24)
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
      7  BOOT      Reboot memory content (0=normal, 1=reboot)          0
      6  FIFO_EN   FIFO enable (0=FIFO disable, 1=enable)              0
      4  HPen      High-pass filter enable (0=disable,1=enable)        0
    3-2  INT1_SEL  INT1 Selection config                              00
    1-0  OUT_SEL   Out selection config                               00 */

    /* Nothing to do ... keep default values */
    /* ------------------------------------------------------------------ */

    // Boot in normal mode, disable FIFO, HPF disabled
    writeRegister(GYRO_REGISTER_CTRL_REG5, 0x00);

  return true;
}


// L3G4200D Temperature sensor output change vs temperature: -1digit/degrCelsius (data representation: 2's complement).
// Value represents difference respect to a reference not specified value.
// So temperature sensor can be used to measure temperature variations: temperarture sensor isn't suitable to return absolute temperatures measures.
// If you run two sequential measures and differentiate them you can get temperature variation.
// This also means that two devices in the same temp conditions can return different outputs.
// Finally, you can use this info to compensate drifts due to temperature changes.
uint8_t L3G4200D::readTemperature(void)
{
    return readRegister(GYRO_REGISTER_OUT_TEMP);
}
// Read raw values
Vector L3G4200D::readRaw()
{
    buffer[0] = readRegister(GYRO_REGISTER_OUT_X_L);
    buffer[1] = readRegister(GYRO_REGISTER_OUT_X_H);

    buffer[2] = readRegister(GYRO_REGISTER_OUT_Y_L);
    buffer[3] = readRegister(GYRO_REGISTER_OUT_Y_H);

    buffer[4] = readRegister(GYRO_REGISTER_OUT_Z_L);
    buffer[5] = readRegister(GYRO_REGISTER_OUT_Z_H);
	
	int16_t x = (int16_t)(buffer[1] << 8 | buffer[0]);
	int16_t y = (int16_t)(buffer[3] << 8 | buffer[2]);
	int16_t z = (int16_t)(buffer[5] << 8 | buffer[4]);
	
	//printf("RAW\nX: %d  Y: %d  Z: %d\n", x, y, z);
	//printf("RAW\nX: %+2.2f  Y: %+2.2f  Z: %+2.2f\n", (float)x, (float)y, (float)z);
	
	r.x = (float)x;
    r.y = (float)y;
    r.z = (float)z;
	//printf("RAW\nX: %+2.2f  Y: %+2.2f  Z: %+2.2f\n", r.x, r.y, r.z);

    return r;
}

// Read normalized values
Vector L3G4200D::readNormalize()
{
    readRaw();

    if (useCalibrate)
    {
        n.x = (r.x - d.x) * dpsPerDigit;
        n.y = (r.y - d.y) * dpsPerDigit;
        n.z = (r.z - d.z) * dpsPerDigit;
    } else
    {
        n.x = r.x * dpsPerDigit;
        n.y = r.y * dpsPerDigit;
        n.z = r.z * dpsPerDigit;
    }

    if (actualThreshold > 0)
    {
        if (abs(n.x) < t.x) n.x = 0;
        if (abs(n.y) < t.y) n.y = 0;
        if (abs(n.z) < t.z) n.z = 0;
    }

    return n;
}

// Get current threshold value
uint8_t L3G4200D::getThreshold(void)
{
    return actualThreshold;
}

// Set treshold value
void L3G4200D::setThreshold(uint8_t multiple)
{
    if (multiple > 0)
    {
	// If not calibrated, need calibrate
	if (!useCalibrate)
	{
	    calibrate();
	}

	// Calculate threshold vectors
	t.x = thresholdX * multiple;
	t.y = thresholdY * multiple;
	t.z = thresholdZ * multiple;
    } else
    {
	// No threshold
	t.x = 0;
	t.y = 0;
	t.z = 0;
    }

    // Remember old threshold value
    actualThreshold = multiple;
}

// Calibrate algorithm
void L3G4200D::calibrate(uint8_t samples)
{
    // Set calibrate
    useCalibrate = true;

    // Reset values
    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;
    float sigmaX = 0;
    float sigmaY = 0;
    float sigmaZ = 0;

    // Read n-samples
    for (uint8_t i = 0; i < samples; ++i)
    {
        readRaw();
        sumX += r.x;
        sumY += r.y;
        sumZ += r.z;

        sigmaX += r.x * r.x;
        sigmaY += r.y * r.y;
        sigmaZ += r.z * r.z;

        delayus(5000);
    }

    // Calculate delta vectors
    d.x = sumX / samples;
    d.y = sumY / samples;
    d.z = sumZ / samples;

    // Calculate threshold vectors
    thresholdX = sqrt((sigmaX / samples) - (d.x * d.x));
    thresholdY = sqrt((sigmaY / samples) - (d.y * d.y));
    thresholdZ = sqrt((sigmaZ / samples) - (d.z * d.z));

    // If already set threshold, recalculate threshold vectors
    if (actualThreshold > 0)
    {
	setThreshold(actualThreshold);
    }
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool L3G4200D::getEvent(sensors_vec_t* event)
{
  bool readingValid = false;

  /* Clear the event */
  memset(event, 0, sizeof(sensors_vec_t));

  event->sensor_id = _sensorID;
  event->type      = SENSOR_TYPE_GYROSCOPE;


  while(!readingValid)
  {
        //readRaw();

        //buffer[0] = readRegister(GYRO_REGISTER_OUT_X_L);
        //buffer[1] = readRegister(GYRO_REGISTER_OUT_X_H);

        //buffer[2] = readRegister(GYRO_REGISTER_OUT_Y_L);
        //buffer[3] = readRegister(GYRO_REGISTER_OUT_Y_H);

        //buffer[4] = readRegister(GYRO_REGISTER_OUT_Z_L);
        //buffer[5] = readRegister(GYRO_REGISTER_OUT_Z_H);

    //selRegister(GYRO_REGISTER_OUT_X_L);
    //if (read(fd, buffer, 6) != 6) {


    //}else{
        //event->x = (float)((int16_t)((buffer[1] << 8) | buffer[0]));
        //event->y = (float)((int16_t)((buffer[3] << 8) | buffer[2]));
        //event->z = (float)((int16_t)((buffer[5] << 8) | buffer[4]));

        //event->x = r.x;
        //event->y = r.y;
        //event->z = r.z;
		
        readNormalize();
        event->x = n.x;// * SENSORS_DPS_TO_RADS;
        event->y = n.y;// * SENSORS_DPS_TO_RADS;
        event->z = n.z;// * SENSORS_DPS_TO_RADS;
		
    //}

    ///* Make sure the sensor isn't saturating if auto-ranging is enabled */
    //if (!_autoRangeEnabled)
    //{
    //  readingValid = true;
    //} else {
    //  /* Check if the sensor is saturating or not */
    //  if ( (event->x >= 32760) | (event->x <= -32760) |
    //       (event->y >= 32760) | (event->y <= -32760) |
    //       (event->z >= 32760) | (event->z <= -32760) )
    //  {
    //    /* Saturating .... increase the range if we can */
    //    switch(_range)
    //    {
    //      case GYRO_RANGE_500DPS:
    //        /* Push the range up to 2000dps */
    //        _range = GYRO_RANGE_2000DPS;
    //        writeRegister(GYRO_REGISTER_CTRL_REG1, 0x00);
    //        writeRegister(GYRO_REGISTER_CTRL_REG1, 0x0F);
    //        writeRegister(GYRO_REGISTER_CTRL_REG4, 0x20);
    //        writeRegister(GYRO_REGISTER_CTRL_REG5, 0x80);
    //        readingValid = false;
    //        // Serial.println("Changing range to 2000DPS");
    //        break;
    //      case GYRO_RANGE_250DPS:
    //        /* Push the range up to 500dps */
    //        _range = GYRO_RANGE_500DPS;
    //        writeRegister(GYRO_REGISTER_CTRL_REG1, 0x00);
    //        writeRegister(GYRO_REGISTER_CTRL_REG1, 0x0F);
    //        writeRegister(GYRO_REGISTER_CTRL_REG4, 0x10);
    //        writeRegister(GYRO_REGISTER_CTRL_REG5, 0x80);
    //        readingValid = false;
    //        // Serial.println("Changing range to 500DPS");
    //        break;
    //      default:
    //        readingValid = true;
    //        break;
    //    }
    //  }
    //  else
    //  {
    //    /* All values are withing range */
        readingValid = true;
    //  }
    //}
  }

  ///* Compensate values depending on the resolution */
  //switch(_range)
  //{
  //  case GYRO_RANGE_250DPS:
  //    event->x *= GYRO_SENSITIVITY_250DPS;
  //    event->y *= GYRO_SENSITIVITY_250DPS;
  //    event->z *= GYRO_SENSITIVITY_250DPS;
  //    break;
  //  case GYRO_RANGE_500DPS:
  //    event->x *= GYRO_SENSITIVITY_500DPS;
  //    event->y *= GYRO_SENSITIVITY_500DPS;
  //    event->z *= GYRO_SENSITIVITY_500DPS;
  //    break;
  //  case GYRO_RANGE_2000DPS:
  //    event->x *= GYRO_SENSITIVITY_2000DPS;
  //    event->y *= GYRO_SENSITIVITY_2000DPS;
  //    event->z *= GYRO_SENSITIVITY_2000DPS;
  //    break;
  //}

  ///* Convert values to rad/s */
  //event->x *= SENSORS_DPS_TO_RADS;
  //event->y *= SENSORS_DPS_TO_RADS;
  //event->z *= SENSORS_DPS_TO_RADS;

  return true;
}

void L3G4200D::delayus (int delay_us)
{
	long int start_time;
	long int time_difference;
	struct timespec gettime_now;

	clock_gettime(CLOCK_REALTIME, &gettime_now);
	start_time = gettime_now.tv_nsec;		//Get nS value
	while (1)
	{
		clock_gettime(CLOCK_REALTIME, &gettime_now);
		time_difference = gettime_now.tv_nsec - start_time;
		if (time_difference < 0)
			time_difference += 1000000000;				//(Rolls over every 1 second)
		if (time_difference > (delay_us * 1000))		//Delay for # nS
			break;
	}
}


/* Class smbusIOException */

L3G4200D::smbusIOException::smbusIOException (const char* m, int err) :
	err(err), msg(m)
{ }

const char* L3G4200D::smbusIOException::what () throw () {
	string m = msg;
	if (err) {
		m += ":\n";
		m += strerror(errno);
	}
	return m.c_str();
}
