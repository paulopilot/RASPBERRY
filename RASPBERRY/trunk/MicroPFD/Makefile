all:

	g++ -Wall -fexceptions -g -I /usr/include/SDL -lm -lSDL_image -lSDL_gfx -lSDL -Iinclude -c /home/share/MicroPFD/main.cpp -o obj/Debug/main.o
	g++ -Wall -fexceptions -g -I /usr/include/SDL -lm -lSDL_image -lSDL_gfx -lSDL -Iinclude -c /home/share/MicroPFD/src/ADXL345.cpp -o obj/Debug/src/ADXL345.o
	g++ -Wall -fexceptions -g -I /usr/include/SDL -lm -lSDL_image -lSDL_gfx -lSDL -Iinclude -c /home/share/MicroPFD/src/BMP085.cpp -o obj/Debug/src/BMP085.o
	g++ -Wall -fexceptions -g -I /usr/include/SDL -lm -lSDL_image -lSDL_gfx -lSDL -Iinclude -c /home/share/MicroPFD/src/Graphics.cpp -o obj/Debug/src/Graphics.o
	g++ -Wall -fexceptions -g -I /usr/include/SDL -lm -lSDL_image -lSDL_gfx -lSDL -Iinclude -c /home/share/MicroPFD/src/HMC5883L.cpp -o obj/Debug/src/HMC5883L.o
	g++ -Wall -fexceptions -g -I /usr/include/SDL -lm -lSDL_image -lSDL_gfx -lSDL -Iinclude -c /home/share/MicroPFD/src/IMU_10DOF.cpp -o obj/Debug/src/IMU_10DOF.o
	g++ -Wall -fexceptions -g -I /usr/include/SDL -lm -lSDL_image -lSDL_gfx -lSDL -Iinclude -c /home/share/MicroPFD/src/L3G4200D.cpp -o obj/Debug/src/L3G4200D.o
	g++ -o bin/Debug/MicroPFD obj/Debug/main.o obj/Debug/src/ADXL345.o obj/Debug/src/BMP085.o obj/Debug/src/Graphics.o obj/Debug/src/HMC5883L.o obj/Debug/src/IMU_10DOF.o obj/Debug/src/L3G4200D.o  -lSDL_image -lSDL_ttf -lSDL_gfx -lSDL

