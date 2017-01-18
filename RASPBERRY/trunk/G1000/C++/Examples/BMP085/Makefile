CC = g++
G1000 = ../../drivers
INCLUDES = -I ../..

all:
	$(CC) $(INCLUDES) BMP085_basic.cpp $(G1000)/BMP085.cpp $(G1000)/I2Cdev.cpp  -o BMP085_basic

clean:
	rm BMP085_basic
