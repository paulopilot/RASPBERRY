#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <sys/time.h>
#include "drivers/I2Cdev.h"
#include "drivers/HMC5883L.h"

int main(int argc, char **argv) {
  printf("HMC5883L 3-axis compass example program\n");
  HMC5883L compass ;
  
  if ( compass.testConnection() ) 
    printf("HMC5883L connection test successful\n") ;
  else {
    fprintf( stderr, "HMC5883L connection test failed! exiting ...\n");
    return 1;
  }
  compass.initialize();
  compass.setMode(HMC5883L_MODE_CONTINUOUS);

  int16_t ax, ay, az;
  while (true) {
    compass.getHeading(&ax, &ay, &az);
    //printf("  x_raw:  0x%04X       y_raw:  0x%04X      z_raw:  0x%04X\r", ax, ay, az);
	printf("  x_raw:  %d       y_raw:  %d      z_raw:  %d\r", ax, ay, az);
    fflush(stdout);
    usleep(200000);
  }
  return 1; 
}


gcc -o gyro_accelerometer_tutorial02 -lm gyro_accelerometer_tutorial02.c `sdl-config --cflags` `sdl-config --libs` -lSDL_image -lSDL_gfx