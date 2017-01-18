#include "HMC5883L.h"
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
void HMC5883L::writeRegister(uint8_t reg, uint8_t value) {



    buffer[0] = reg;
    buffer[1] = value;

    if (write(fd, buffer, 2) != 2)
    {
        char str[32];
        sprintf(str, "writeToDevice() 0x%x", fd);
        throw HMC5883L::smbusIOException (
            str,
            errno
        );
    }
}

int HMC5883L::selRegister(uint8_t reg) {

    buffer[0] = reg;

    int result = write(fd, buffer, 1);

    if (result != 1)
    {
        char str[32];
        sprintf(str, "writeToDevice() 0x%x", fd);
        throw HMC5883L::smbusIOException (
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
uint8_t HMC5883L::readRegister(uint8_t reg) {

    int ret_val;

    selRegister(reg);

    //ret_val = i2c_smbus_read_byte_data(fd, reg);

    ret_val = read(fd, buffer, 1);

    if (ret_val != 1) {
        char str[32];
		sprintf(str, "readRegister() 0x%x", reg);
		throw HMC5883L::smbusIOException (
			str,
			errno
		);
    }else{
        ret_val = buffer[0];
    }

    return (uint8_t)ret_val;

}

/**************************************************************************/




HMC5883L::HMC5883L(string dev, int addr)  :
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

	// write CONFIG_A register
    writeRegister(HMC5883L_RA_CONFIG_A,
        (HMC5883L_AVERAGING_8 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1)) |
        (HMC5883L_RATE_15     << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)) |
        (HMC5883L_BIAS_NORMAL << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1)));

    // write CONFIG_B register
    setGain(HMC5883L_GAIN_1090);

    // write MODE register
    //setMode(HMC5883L_MODE_SINGLE);

    writeRegister(HMC5883L_RA_CONFIG_B, 32);
    writeRegister(HMC5883L_RA_MODE, HMC5883L_MODE_CONTINUOUS);
}

HMC5883L::~HMC5883L()
{
    close(fd);
}

// CONFIG_A register

void HMC5883L::getREG_A()
{
    uint8_t regA = readRegister(HMC5883L_RA_CONFIG_A);
    uint8_t t_regA = regA;

    bias = t_regA & 0x03;
    t_regA = regA;

    data_rate = (t_regA >> 2) & 0x07;
    t_regA = regA;

    samples = (t_regA >> 5) & 0x03;

}

void HMC5883L::setREG_A()
{
    uint8_t regA = 0;

    regA = samples << 5;
    regA = regA | (data_rate << 2);
    regA = regA | bias;

    writeRegister(HMC5883L_RA_CONFIG_A, regA);

}

/** Get number of samples averaged per measurement.
 * @return Current samples averaged per measurement (0-3 for 1/2/4/8 respectively)
 * @see HMC5883L_AVERAGING_8
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_AVERAGE_BIT
 * @see HMC5883L_CRA_AVERAGE_LENGTH
 */
uint8_t HMC5883L::getSampleAveraging() {

    getREG_A();
    return samples;
}
/** Set number of samples averaged per measurement.
 * @param averaging New samples averaged per measurement setting(0-3 for 1/2/4/8 respectively)
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_AVERAGE_BIT
 * @see HMC5883L_CRA_AVERAGE_LENGTH
 */
void HMC5883L::setSampleAveraging(uint8_t averaging) {

    samples = averaging;
    setREG_A();
}

/** Get data output rate value.
 * The Table below shows all selectable output rates in continuous measurement
 * mode. All three channels shall be measured within a given output rate. Other
 * output rates with maximum rate of 160 Hz can be achieved by monitoring DRDY
 * interrupt pin in single measurement mode.
 *
 * Value | Typical Data Output Rate (Hz)
 * ------+------------------------------
 * 0     | 0.75
 * 1     | 1.5
 * 2     | 3
 * 3     | 7.5
 * 4     | 15 (Default)
 * 5     | 30
 * 6     | 75
 * 7     | Not used
 *
 * @return Current rate of data output to registers
 * @see HMC5883L_RATE_15
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_RATE_BIT
 * @see HMC5883L_CRA_RATE_LENGTH
 */

uint8_t HMC5883L::getDataRate() {
    getREG_A();
    return data_rate;
}

/** Set data output rate value.
 * @param rate Rate of data output to registers
 * @see getDataRate()
 * @see HMC5883L_RATE_15
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_RATE_BIT
 * @see HMC5883L_CRA_RATE_LENGTH
 */
void HMC5883L::setDataRate(uint8_t rate) {
    data_rate = rate;
    setREG_A();
}

/** Get measurement bias value.
 * @return Current bias value (0-2 for normal/positive/negative respectively)
 * @see HMC5883L_BIAS_NORMAL
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_BIAS_BIT
 * @see HMC5883L_CRA_BIAS_LENGTH
 */
uint8_t HMC5883L::getMeasurementBias() {
    getREG_A();
    return bias;
}

/** Set measurement bias value.
 * @param bias New bias value (0-2 for normal/positive/negative respectively)
 * @see HMC5883L_BIAS_NORMAL
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_BIAS_BIT
 * @see HMC5883L_CRA_BIAS_LENGTH
 */
void HMC5883L::setMeasurementBias(uint8_t t_bias) {
    bias = t_bias;
    setREG_A();
}
// CONFIG_B register


/** Get magnetic field gain value.
 * The table below shows nominal gain settings. Use the "Gain" column to convert
 * counts to Gauss. Choose a lower gain value (higher GN#) when total field
 * strength causes overflow in one of the data output registers (saturation).
 * The data output range for all settings is 0xF800-0x07FF (-2048 - 2047).
 *
 * Value | Field Range | Gain (LSB/Gauss)
 * ------+-------------+-----------------
 * 0     | +/- 0.88 Ga | 1370
 * 1     | +/- 1.3 Ga  | 1090 (Default)
 * 2     | +/- 1.9 Ga  | 820
 * 3     | +/- 2.5 Ga  | 660
 * 4     | +/- 4.0 Ga  | 440
 * 5     | +/- 4.7 Ga  | 390
 * 6     | +/- 5.6 Ga  | 330
 * 7     | +/- 8.1 Ga  | 230
 *
 * @return Current magnetic field gain value
 * @see HMC5883L_GAIN_1090
 * @see HMC5883L_RA_CONFIG_B
 * @see HMC5883L_CRB_GAIN_BIT
 * @see HMC5883L_CRB_GAIN_LENGTH
 */
uint8_t HMC5883L::getGain() {
    uint8_t t_gain = readRegister(HMC5883L_RA_CONFIG_B);
    t_gain = (t_gain & 0xE0) >> 5;
    return t_gain;
}

/** Set magnetic field gain value.
 * @param gain New magnetic field gain value
 * @see getGain()
 * @see HMC5883L_RA_CONFIG_B
 * @see HMC5883L_CRB_GAIN_BIT
 * @see HMC5883L_CRB_GAIN_LENGTH
 */
void HMC5883L::setGain(uint8_t gain) {
    // use this method to guarantee that bits 4-0 are set to zero, which is a
    // requirement specified in the datasheet; it's actually more efficient than
    // using the I2Cdev.writeBits method
    writeRegister( HMC5883L_RA_CONFIG_B, gain << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1));
	
	switch(gain)
    {
	case HMC5883L_GAIN_1370:
	    mgPerDigit = 0.73f;
	    break;

	case HMC5883L_GAIN_1090:
	    mgPerDigit = 0.92f;
	    break;

	case HMC5883L_GAIN_820:
	    mgPerDigit = 1.22f;
	    break;

	case HMC5883L_GAIN_660:
	    mgPerDigit = 1.52f;
	    break;

	case HMC5883L_GAIN_440:
	    mgPerDigit = 2.27f;
	    break;

	case HMC5883L_GAIN_390:
	    mgPerDigit = 2.56f;
	    break;

	case HMC5883L_GAIN_330:
	    mgPerDigit = 3.03f;
	    break;

	case HMC5883L_GAIN_220:
	    mgPerDigit = 4.35f;
	    break;

	default:
		mgPerDigit = 0.92f;
	    break;
    }
}

// MODE register

/** Get measurement mode.
 * In continuous-measurement mode, the device continuously performs measurements
 * and places the result in the data register. RDY goes high when new data is
 * placed in all three registers. After a power-on or a write to the mode or
 * configuration register, the first measurement set is available from all three
 * data output registers after a period of 2/fDO and subsequent measurements are
 * available at a frequency of fDO, where fDO is the frequency of data output.
 *
 * When single-measurement mode (default) is selected, device performs a single
 * measurement, sets RDY high and returned to idle mode. Mode register returns
 * to idle mode bit values. The measurement remains in the data output register
 * and RDY remains high until the data output register is read or another
 * measurement is performed.
 *
 * @return Current measurement mode
 * @see HMC5883L_MODE_CONTINUOUS
 * @see HMC5883L_MODE_SINGLE
 * @see HMC5883L_MODE_IDLE
 * @see HMC5883L_RA_MODE
 * @see HMC5883L_MODEREG_BIT
 * @see HMC5883L_MODEREG_LENGTH
 */
uint8_t HMC5883L::getMode() {


    uint8_t t_mode = readRegister(HMC5883L_RA_MODE);
    t_mode = t_mode & 0x03;
    return t_mode;

}
/** Set measurement mode.
 * @param newMode New measurement mode
 * @see getMode()
 * @see HMC5883L_MODE_CONTINUOUS
 * @see HMC5883L_MODE_SINGLE
 * @see HMC5883L_MODE_IDLE
 * @see HMC5883L_RA_MODE
 * @see HMC5883L_MODEREG_BIT
 * @see HMC5883L_MODEREG_LENGTH
 */
void HMC5883L::setMode(uint8_t newMode) {
    // use this method to guarantee that bits 7-2 are set to zero, which is a
    // requirement specified in the datasheet; it's actually more efficient than
    // using the I2Cdev.writeBits method
    writeRegister( HMC5883L_RA_MODE, newMode << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
    mode = newMode; // track to tell if we have to clear bit 7 after a read
}


// DATA* registers

void HMC5883L::setOffset(int xo, int yo)
{
    xOffset = xo;
    yOffset = yo;
}

/** Get 3-axis heading measurements.
 * In the event the ADC reading overflows or underflows for the given channel,
 * or if there is a math overflow during the bias measurement, this data
 * register will contain the value -4096. This register value will clear when
 * after the next valid measurement is made. Note that this method automatically
 * clears the appropriate bit in the MODE register if Single mode is active.
 * @param x 16-bit signed integer container for X-axis heading
 * @param y 16-bit signed integer container for Y-axis heading
 * @param z 16-bit signed integer container for Z-axis heading
 * @see HMC5883L_RA_DATAX_H
 */
Vector HMC5883L::readRaw() {

    if (mode == HMC5883L_MODE_SINGLE)
        writeRegister(HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));

    selRegister(HMC5883L_RA_DATAX_H);

    if (read(fd, buffer, 6) != 6) {


    }else{
		int16_t x = (int16_t)(buffer[0] << 8 | buffer[1]);
		int16_t y = (int16_t)(buffer[4] << 8 | buffer[5]);
		int16_t z = (int16_t)(buffer[2] << 8 | buffer[3]);
		
        r.x = (float)(x - xOffset);
        r.y = (float)(y - yOffset);
        r.z = (float)z;
    }
	
	return r;
 }

float HMC5883L::getHeading(float dmg) {

    readNormalize();
    float heading = atan2(n.y, n.x);
	
	heading -= dmg / (180/M_PI);
	
	// Correct for heading < 0deg and heading > 360deg
	if (heading < 0)
	{
		heading += 2 * M_PI;
	}

	if (heading > 2 * M_PI)
	{
		heading -= 2 * M_PI;
	}

    return  heading * (180/M_PI);

}

Vector HMC5883L::readNormalize() {

    readRaw();
    n.x = r.x * mgPerDigit;
	n.y = r.y * mgPerDigit;
	n.z = r.z * mgPerDigit;
	
	return n;

}

bool HMC5883L::getEvent(sensors_vec_t *event) {
    memset(event, 0, sizeof(sensors_vec_t));

    readNormalize();

    //event->version = sizeof(sensors_vec_t);
    event->type = SENSOR_TYPE_MAGNETIC_FIELD;
    //event->timestamp = 0;
    event->x = n.x;
    event->y = n.y;
    event->z = n.z;

    return true;
}

/** Get X-axis heading measurement.
 * @return 16-bit signed integer with X-axis heading
 * @see HMC5883L_RA_DATAX_H
 */
float HMC5883L::readRawX() {

    readRaw();

    return r.x;
}
/** Get Y-axis heading measurement.
 * @return 16-bit signed integer with Y-axis heading
 * @see HMC5883L_RA_DATAY_H
 */
float HMC5883L::readRawY() {

    readRaw();

    return r.y;
}
/** Get Z-axis heading measurement.
 * @return 16-bit signed integer with Z-axis heading
 * @see HMC5883L_RA_DATAZ_H
 */
float HMC5883L::readRawZ() {
	
	readRaw();
	
    return r.z;
}


/* Class smbusIOException */

HMC5883L::smbusIOException::smbusIOException (const char* m, int err) :
	err(err), msg(m)
{ }

const char* HMC5883L::smbusIOException::what () throw () {
	string m = msg;
	if (err) {
		m += ":\n";
		m += strerror(errno);
	}
	return m.c_str();
}
