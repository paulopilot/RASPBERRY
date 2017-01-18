

#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>

#include <math.h>
#include <signal.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>

#include "BMP085.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "L3G4200D.h"

#include "sensor.h"
#include "IMU_10DOF.h"
#include "Graphics.h"

/***********************************
https://github.com/jarzebski/Arduino-L3G4200D
http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-l3g4200d.html
https://www.parallax.com/sites/default/files/downloads/27911-Gyroscope-3-Axis-L3G4200D-Guide-v1.1.pdf

g++ -Wall -fexceptions -g -I /usr/include/SDL -lm -lSDL_image -lSDL_gfx -lSDL -Iinclude -c /home/share/MicroPFD/main.cpp -o obj/Debug/main.o
g++ -Wall -fexceptions -g -I /usr/include/SDL -lm -lSDL_image -lSDL_gfx -lSDL -Iinclude -c /home/share/MicroPFD/src/ADXL345.cpp -o obj/Debug/src/ADXL345.o
g++ -Wall -fexceptions -g -I /usr/include/SDL -lm -lSDL_image -lSDL_gfx -lSDL -Iinclude -c /home/share/MicroPFD/src/BMP085.cpp -o obj/Debug/src/BMP085.o
g++ -Wall -fexceptions -g -I /usr/include/SDL -lm -lSDL_image -lSDL_gfx -lSDL -Iinclude -c /home/share/MicroPFD/src/Graphics.cpp -o obj/Debug/src/Graphics.o
g++ -Wall -fexceptions -g -I /usr/include/SDL -lm -lSDL_image -lSDL_gfx -lSDL -Iinclude -c /home/share/MicroPFD/src/HMC5883L.cpp -o obj/Debug/src/HMC5883L.o
g++ -Wall -fexceptions -g -I /usr/include/SDL -lm -lSDL_image -lSDL_gfx -lSDL -Iinclude -c /home/share/MicroPFD/src/IMU_10DOF.cpp -o obj/Debug/src/IMU_10DOF.o
g++ -Wall -fexceptions -g -I /usr/include/SDL -lm -lSDL_image -lSDL_gfx -lSDL -Iinclude -c /home/share/MicroPFD/src/L3G4200D.cpp -o obj/Debug/src/L3G4200D.o
g++ -o bin/Debug/MicroPFD obj/Debug/main.o obj/Debug/src/ADXL345.o obj/Debug/src/BMP085.o obj/Debug/src/Graphics.o obj/Debug/src/HMC5883L.o obj/Debug/src/IMU_10DOF.o obj/Debug/src/L3G4200D.o  -lSDL_image -lSDL_ttf -lSDL_gfx -lSDL


**************************************/



using namespace std;

void usage () {
	cout << "- USAGE -\n"
		"All arguments are optional.\n"
		"1st arg, oversampling setting: 0-3 (default 1)\n"
		"2nd arg, high res pressure mode: 0 or 1 (default 1)\n";
	exit(0);
}

ADXL345 		accel;
HMC5883L 		compass;
BMP085 			bmp;
L3G4200D 		gyro;
IMU_10DOF dof;//  = IMU_10DOF();

Graphics pfd; // = Graphics();


//*****************************************************
//*****************************************************
//********** DELAY FOR # uS WITHOUT SLEEPING **********
//*****************************************************
//*****************************************************
//Using delayMicroseconds lets the linux scheduler decide to jump to another process.  Using this function avoids letting the
//scheduler know we are pausing and provides much faster operation if you are needing to use lots of delays.
void delayus (int delay_us)
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

int main (int argc, const char** argv) {
// Set defaults and check for arguments.


	sensors_vec_t	accel_event;
	sensors_vec_t	mag_event;
	sensors_vec_t	bmp_event;
    sensors_vec_t	gyro_event;
	sensors_event_t imu_event;

	imu_event.q.w = 1.0f;
	imu_event.q.x = 0.0f;
    imu_event.q.y = 0.0f;
    imu_event.q.z = 0.0f;

    //accel.offsetCalibration();

	//BMP085::OversamplingSetting oss = BMP085::OSS_STANDARD;
	bool hires = true;
	//if (argc > 1) {
	//	int n;
	//	istringstream(argv[1]) >> n;
	//	switch (n) {
	//		case 0:
	//			oss = BMP085::OSS_LOW;
	//			break;
	//		case 1: break;
	//		case 2:
	//			oss = BMP085::OSS_HIGH;
	//			break;
	//		case 3:
	//			oss = BMP085::OSS_ULTRAHIGH;
	//			break;
	//		default: usage();
	//	}
	//	if (argc > 2) {
	//		istringstream(argv[2]) >> n;
	//		switch (n) {
	//			case 0:
	//				hires = false;
	//				break;
	//			case 1: break;
	//		default: usage();
	//		}
	//	}
	//}

	gyro.begin();
	gyro.calibrate(100);

	//acell.begin();

    // Create object.
	;
	// For rev. 1 Model B pis:
	// BMP085 *bcm = new BMP085(oss, "/dev/i2c-0");
	if (!bmp.ok) {
		cerr << bmp.err << endl;
		return 1;
	}
	bmp.hiRes = hires;

	for(int i=2;i>=0;i--){
        cout << "\rStarting in " << i << flush;
        sleep(1);
	}
	cout << "\n";

	//if (pfd.initSDL() == 1) {
    //    cout << "Error starting SDL graphics interface.\n";
    //    return 1;
    //}else{
    //    pfd.initScreen();
    //}

    // Take reading, report, and repeat ad infititum.
	try {
        BMP085::reading data = bmp.getBoth();

		cout << "Relative altitude at " << data.kPa << " kPa: "
			<< BMP085::getRelativeAltitude(data.kPa) << "m.\n";

        //float AccelMinX = 0;
        //float AccelMaxX = 0;
        //float AccelMinY = 0;
        //float AccelMaxY = 0;
        //float AccelMinZ = 0;
        //float AccelMaxZ = 0;
        //float offSetX = 0;
        //float offSetY = 0;
        //float offSetZ = 0;
        //float gainX = 0;
        //float gainY = 0;
        //float gainZ = 0;
		
		
		//float MagMinX = 0;
        //float MagMaxX = 0;
        //float MagMinY = 0;
        //float MagMaxY = 0;

        //accel.setRange(ADXL345_RANGE_16_G);
		
		compass.setSampleAveraging(HMC5883L_AVERAGING_8);
		compass.setDataRate(HMC5883L_RATE_30);
		compass.setOffset(0, 0); //139 , 42

		accel.getEvent(&accel_event);
        compass.getEvent(&mag_event);
        bmp.getEvent(&bmp_event);
        gyro.getEvent(&gyro_event);
		
        imu_event.accel = accel_event;
        imu_event.compass = mag_event;
        imu_event.gyro = gyro_event;

		while (1) {

			//cout << "\rTemperature: " << data.celcius << " °C    "
			//	<< "Pressure: " << data.kPa << " kPa         "
            //    << "Pitch=" << orientation.roll << flush;
			//data = bmp.getBoth();
			//accelData = accel.getXYZ();
			////dof.getOrientation(&accelData, &orientation);
            //accel_event.orientation = orientation;
            //accel_event.temperature = data.celcius;

            //dof.MadgwickQuaternionUpdate(&imu_event);
			//printf("X: %+2.2f Y: %+2.2f Z: %+2.2f\n", accel_event.x, accel_event.y, accel_event.z);
			//printf("X: %+2.2f Y: %+2.2f Z: %+2.2f\n", imu_event.accel.x, imu_event.accel.y, imu_event.accel.z);
			
			
			dof.tiltCompensate(&imu_event);

            //if (accel_event.x < AccelMinX) AccelMinX = accel_event.x;
            //if (accel_event.x > AccelMaxX) AccelMaxX = accel_event.x;

            //if (accel_event.y < AccelMinY) AccelMinY = accel_event.y;
            //if (accel_event.y > AccelMaxY) AccelMaxY = accel_event.y;

            //if (accel_event.z < AccelMinZ) AccelMinZ = accel_event.z;
            //if (accel_event.z > AccelMaxZ) AccelMaxZ = accel_event.z;

            //offSetX = 0.5 * (AccelMaxX + AccelMinX);
            //offSetY = 0.5 * (AccelMaxY + AccelMinY);
            //offSetZ = 0.5 * (AccelMaxZ + AccelMinZ);

            //gainX = 0.5 * (AccelMaxX - AccelMinX);
            //gainY = 0.5 * (AccelMaxY - AccelMinY);
            //gainZ = 0.5 * (AccelMaxZ - AccelMinZ);

			//printf("\nIMU\n");
            printf ("IMU Pitch..: %+3.1f Roll: %+3.1f Yaw: %+3.1f\n", imu_event.orientation.pitch, imu_event.orientation.roll, imu_event.orientation.yaw);
			
            //printf("\nAccel\n");
            //printf("X: %+2.2f Y: %+2.2f Z: %+2.2f\n", accel_event.x, accel_event.y, accel_event.z);
			//sleep(60);
            //printf("Min X: %+2.2f Max X: %+2.2f offSet: %+2.2f Gain: %+2.2f\n", AccelMinX, AccelMaxX, offSetX, gainX);
            //printf("Min Y: %+2.2f Max Y: %+2.2f offSet: %+2.2f Gain: %+2.2f\n", AccelMinY, AccelMaxY, offSetY, gainY);
            //printf("Min Z: %+2.2f Max Z: %+2.2f offSet: %+2.2f Gain: %+2.2f\n", AccelMinZ, AccelMaxZ, offSetZ, gainZ);
            //here's the magic ...
            float pitch = (atan2(accel_event.x,(sqrt(accel_event.y*accel_event.y+accel_event.z*accel_event.z))) * 180.0) / M_PI;
            float roll  = (atan2(accel_event.y,(sqrt(accel_event.x*accel_event.x+accel_event.z*accel_event.z))) * 180.0) / M_PI;

            printf("Accel Pitch: %+3.1f Roll: %+3.1f\n", pitch, roll);

            //printf("\nGyro\n");
			
			
			//printf("\nMag\n");
			//Vector mag = compass.readRaw();
			//if (mag.x < MagMinX) MagMinX = mag.x;
            //if (mag.x > MagMaxX) MagMaxX = mag.x;

            //if (mag.y < MagMinY) MagMinY = mag.y;
            //if (mag.y > MagMaxY) MagMaxY = mag.y; 
			
			// Calculate offsets
			//offSetX = (MagMaxX + MagMinX)/2;
			//offSetY = (MagMaxY + MagMinY)/2;
			//printf("Nor X: %+2.2f Y: %+2.2f Z: %+2.2f\n", mag_event.x, mag_event.y, mag_event.z);
			//printf("Raw X: %+2.2f Y: %+2.2f Z: %+2.2f\n", mag.x, mag.y, mag.z);
			//printf("Min X: %+2.2f Max X: %+2.2f offSet: %+2.2f\n", MagMinX, MagMaxX, offSetX);
            //printf("Min Y: %+2.2f Max Y: %+2.2f offSet: %+2.2f\n", MagMinY, MagMaxY, offSetY);
			//printf("Mag Heading: %3.0f\n", compass.getHeading(-21.13));
			printf("Mag Heading: %3.0f\n", compass.getHeading());
		
			/* Read the accelerometer and magnetometer */
            accel.getEvent(&accel_event);
            compass.getEvent(&mag_event);
            bmp.getEvent(&bmp_event);
            gyro.getEvent(&gyro_event);
			

            imu_event.accel = accel_event;
            imu_event.compass = mag_event;
            imu_event.gyro = gyro_event;
			


			delayus(500000);
            //sleep(1);

            //if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
            //{
            //    //pfd.drawScreen(&orientation);
            //    /* 'orientation' should have valid .roll and .pitch fields */
            //    cout << "\rRoll: " << orientation.roll << " °    "
            //         << "Pitch: " << orientation.pitch << " º    "
            //         << "Heading=" << orientation.heading << " º    "
            //         << "Temp:" << bmp.getCelcius() << " ºC    "
            //         << "Alt:" << bmp.getRelativeAltitude() << " m    " << flush;
            //}
        }
	} catch (BMP085::smbusIOException &ex) {
		cerr << "SMBus I/O Error:\n" << ex.what();
	}


	return 0;
}


