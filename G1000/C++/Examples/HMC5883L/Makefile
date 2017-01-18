CC = g++
G1000 = ../../drivers
INCLUDES = -I ../..

all:
	$(CC) $(INCLUDES) HMC5883L_test.cpp $(G1000)/HMC5883L.cpp $(G1000)/I2Cdev.cpp  -o HMC5883L_test

clean:
	rm HMC5883L_test
