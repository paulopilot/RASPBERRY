#include <SDL/SDL.h>
#include <SDL/SDL_image.h>
#include <SDL/SDL_ttf.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>

//gcc -I /usr/include/SDL -o test test.c -lm -lSDL_image -lSDL_ttf -lSDL_gfx -lSDL 

int startSDL();
int graphics(float roll, float pitch, float yaw);

SDL_Surface* screen = NULL;

SDL_Surface* horizonImage = NULL;
SDL_Surface* backGroundImage = NULL;
SDL_Surface* backDropImage = NULL;
SDL_Surface* compassImage = NULL;
SDL_Surface* upperCompassImage = NULL;
SDL_Surface* needleImage = NULL;
SDL_Surface* planeImage = NULL;
SDL_Surface* iasImage = NULL;
SDL_Surface* pitchImage = NULL;
SDL_Surface* speedImage = NULL;
SDL_Surface* altitudeImage = NULL;

SDL_Surface* rotationHorizon = NULL;
SDL_Surface* rotationCompass = NULL;
SDL_Surface* rotationUpperCompass = NULL;
SDL_Surface* rotationNeedle = NULL;
SDL_Surface* rotationPitch = NULL;

SDL_Rect backGroundPosition;

TTF_Font *text_font;// =  TTF_OpenFont("/usr/share/fonts/truetype/freefont/FreeSans.ttf", 10);

SDL_VideoInfo* videoInfo;

int video_source;
float roll;
float pitch;
float yaw;

int main(int argc, char** argv) {
	
	roll = 0;
	pitch = 0;
	yaw = 0;
	
	if (argc > 1) {
		video_source = atoi(argv[1]);
		
		roll = atof(argv[2]);
		
	}
	
	if (argc > 2) {
		pitch = atof(argv[3]);
		
	}
	
	if (argc > 3) {
		yaw = atof(argv[4]);
		
	}
	
	if ((video_source > 1) || (video_source < 0)) {
		video_source = 0;
	}
	
	printf("\n");
	
	startSDL();
	
	TTF_Init();
	//text_font =  TTF_OpenFont("/usr/share/fonts/truetype/freefont/FreeSans.ttf", 10);
	//text_font =  TTF_OpenFont("/usr/share/fonts/truetype/freefont/FreeMono.ttf", 10);
	text_font =  TTF_OpenFont("/usr/share/fonts/truetype/terminus/TerminusTTF-4.40.1.ttf", 12);
	if (text_font == NULL) {
		printf("\nCould not load font\n");
		return 0;
	}
	
	//int i;
	//for (i=0;i<60;i+=1){
	//	pitch = (float)i;
	//	graphics(roll, pitch, yaw);
	//}
	
	//for (i=60;i>-60;i-=1){
	//	pitch = (float)i;
	//	graphics(roll, pitch, yaw);
	//}
	//rool = pitch = yaw = 0;
	
	graphics(roll, pitch, yaw);

	getchar();

	SDL_Quit();
	return 0;
}

int startSDL()
{
	//fb1 = small TFT.   fb0 = HDMI/RCA output
	if (video_source == 0)
		putenv("SDL_FBDEV=/dev/fb0");	// HDMI
	if (video_source == 1)
		putenv("SDL_FBDEV=/dev/fb1");	// TFT

	//Initialize  SDL and disable mouse
    SDL_Init(SDL_INIT_VIDEO);
    SDL_ShowCursor(SDL_DISABLE);

	//Get information about the current video device.  E.g. resolution and bits per pixal
    videoInfo = SDL_GetVideoInfo ();

	//Setup a Video mode.
    screen = SDL_SetVideoMode(videoInfo->current_w, videoInfo->current_h, videoInfo->vfmt->BitsPerPixel, SDL_SWSURFACE );
	
    if ( screen == NULL ) {
        fprintf(stderr, "Unable to setvideo: %s\n", SDL_GetError());
        exit(1);
    }
	
	//TTF_Init();

	//Load image.
	SDL_Surface* tempImage =  IMG_Load("horizon.png");
    if (tempImage == NULL){
        printf("error loading horizon image\n");
        SDL_Quit();
        exit(1);
    }
	//Convert the image to a format we can use.
	horizonImage = SDL_DisplayFormatAlpha(tempImage);
	
	//Load image.
	tempImage =  IMG_Load("background.png");
    if (tempImage == NULL){
        printf("error loading background image\n");
        SDL_Quit();
        exit(1);
    }
	//Convert the image to a format we can use.
	backGroundImage = SDL_DisplayFormatAlpha(tempImage);
	
	
	//Load image.
	tempImage =  IMG_Load("backdrop.png");
    if (tempImage == NULL){
        printf("error loading backdrop image\n");
        SDL_Quit();
        exit(1);
    }
	//Convert the image to a format we can use.
	backDropImage = SDL_DisplayFormatAlpha(tempImage);
	
	//Load image.
	tempImage =  IMG_Load("compassrose.png");
    if (tempImage == NULL){
        printf("error loading compassrose image\n");
        SDL_Quit();
        exit(1);
    }
	//Convert the image to a format we can use.
	compassImage = SDL_DisplayFormatAlpha(tempImage);
	
	//Load image.
	tempImage =  IMG_Load("uppercompass.png");
    if (tempImage == NULL){
        printf("error loading compassrose image\n");
        SDL_Quit();
        exit(1);
    }
	//Convert the image to a format we can use.
	upperCompassImage = SDL_DisplayFormatAlpha(tempImage);
	
	
	//Load image.
	tempImage =  IMG_Load("needle.png");
    if (tempImage == NULL){
        printf("error loading needle image\n");
        SDL_Quit();
        exit(1);
    }
	//Convert the image to a format we can use.
	needleImage = SDL_DisplayFormatAlpha(tempImage);
	
	
	//Load image.
	tempImage =  IMG_Load("plane.png");
    if (tempImage == NULL){
        printf("error loading plane image\n");
        SDL_Quit();
        exit(1);
    }
	//Convert the image to a format we can use.
	planeImage = SDL_DisplayFormatAlpha(tempImage);
	
	//Load image.
	tempImage =  IMG_Load("IAS_image.png");
    if (tempImage == NULL){
        printf("error loading IAS indicator image\n");
        SDL_Quit();
        exit(1);
    }
	//Convert the image to a format we can use.
	iasImage = SDL_DisplayFormatAlpha(tempImage);
	
	//Load image.
	tempImage =  IMG_Load("pitchImage.png");
    if (tempImage == NULL){
        printf("error loading Pitch indicator image\n");
        SDL_Quit();
        exit(1);
    }
	//Convert the image to a format we can use.
	pitchImage = SDL_DisplayFormatAlpha(tempImage);
	
	//Load image.
	tempImage =  IMG_Load("speed_scale.png");
    if (tempImage == NULL){
        printf("error loading Speed indicator image\n");
        SDL_Quit();
        exit(1);
    }
	//Convert the image to a format we can use.
	speedImage = SDL_DisplayFormatAlpha(tempImage);
	
	//Load image.
	tempImage =  IMG_Load("altitude_scale.png");
    if (tempImage == NULL){
        printf("error loading Speed indicator image\n");
        SDL_Quit();
        exit(1);
    }
	//Convert the image to a format we can use.
	altitudeImage = SDL_DisplayFormatAlpha(tempImage);
	
	
	//SDL_free(tempImage);
	
	backGroundPosition.x = (videoInfo->current_w/2)-(backGroundImage->w/2);
	backGroundPosition.y = (videoInfo->current_h/2)-(backGroundImage->h/2);
}


int graphics(float roll, float pitch, float yaw)
{
	float speed = 0;
	float altitude = 0;
	float temperature = 0;
	float heading = 0;
	
	printf("Roll=%3.1f\t", roll);
	printf("Pitch=%3.1f\t", pitch);
	printf("Yaw=%3.1f\t", yaw);
	
	if (pitch > 80.0) pitch = 80.00;
	if (pitch < -80.0) pitch = -80.00;
	
	pitch *= 2.4;
	pitch *= -1;
	


	//Set all pixels to black to clear the last image.
    SDL_FillRect(screen,NULL,0x000000);

	/***************  HORIZON IMAGE ****************/
	float x,y;
	x = (float)(videoInfo->current_w/2)-(horizonImage->w/2) -21;
	//y = (float)(videoInfo->current_h/2)-(horizonImage->h/2) -31;
	y = -126;

	SDL_Rect horizonPosition={(int)x,(int)y,0,0};
	
	rotationHorizon = rotozoomSurface(horizonImage, roll, 1.0, 0.0);
	
	x += sin(roll*M_PI/180.0)*2.0;
	y -= cos(roll*M_PI/180.0)*2.0;
	
	horizonPosition.x -=  (rotationHorizon->w/2)-(horizonImage->w/2);
    horizonPosition.y -=  (rotationHorizon->h/2)-(horizonImage->h/2);
	
	SDL_Rect camera;
	camera.x =  0;
	camera.y =  (int)pitch + 25.0;  // pitch offset
	camera.w =  640;
    camera.h =  480;
	
	/***************  COMPASS ****************/
	SDL_Rect compassPosition;
	//SDL_Rect compassPosition={640,480,0,0};
	compassPosition.x = (videoInfo->current_w/2)-(compassImage->w/2) - 21;
	compassPosition.y = (videoInfo->current_h/2)-(compassImage->h/2) + 61;
	
	//printf("x=%d\ty=%d\n", (videoInfo->current_w/2)-(compassImage->w/2), (videoInfo->current_h/2)-(compassImage->h/2));
	
	rotationCompass = rotozoomSurface(compassImage, yaw, 1.0, 0.0);
	
	compassPosition.x -= rotationCompass->w/2-compassImage->w/2;
    compassPosition.y -= rotationCompass->h/2-compassImage->h/2;
	
	
	/***************  UPPER COMPASS ****************/
	x = (float)(videoInfo->current_w/2)-(upperCompassImage->w/2) -21; //20
	y = (float)(videoInfo->current_h/2)-(upperCompassImage->h/2) -38; //38

	SDL_Rect upperCompassPosition={(int)x,(int)y,0,0};
	
	rotationUpperCompass = rotozoomSurface(upperCompassImage, roll, 1.0, 0.0);
	
	x += sin(roll*M_PI/180.0)*2.0;
	
	upperCompassPosition.x -=  (rotationUpperCompass->w/2)-(upperCompassImage->w/2);
    upperCompassPosition.y -=  (rotationUpperCompass->h/2)-(upperCompassImage->h/2);
	
	/***************  NEEDLE ****************/
	SDL_Rect needlePosition;
	//SDL_Rect compassPosition={640,480,0,0};
	needlePosition.x = (videoInfo->current_w/2)-(needleImage->w/2) - 21;
	needlePosition.y = (videoInfo->current_h/2)-(needleImage->h/2) + 62;
	
	float hdg_tmp = heading * -1;
	
	rotationNeedle = rotozoomSurface(needleImage, hdg_tmp, 1.0, 0.0);
	
	needlePosition.x -= rotationNeedle->w/2-needleImage->w/2;
    needlePosition.y -= rotationNeedle->h/2-needleImage->h/2;
	/***********************************************/
	
	
	SDL_Rect planePosition;
	//SDL_Rect compassPosition={640,480,0,0};
	planePosition.x = (videoInfo->current_w/2)-(planeImage->w/2) - 21;
	planePosition.y = (videoInfo->current_h/2)-(planeImage->h/2) + 62;
	
	SDL_Rect iasPosition={43,79,0,0};

	
	/***************  PITCH IMAGE ****************/
	
	x = (float)(videoInfo->current_w/2)-(pitchImage->w/2) -21;
	//y = (float)(videoInfo->current_h/2)-(pitchImage->h/2) -31;
	y = 35;

	SDL_Rect pitchPosition={(int)x,(int)y,0,0};
	
	rotationPitch = rotozoomSurface(pitchImage, roll, 1.0, 0.0);
	
	x += sin(roll*M_PI/180.0)*2.0;
	y -= cos(roll*M_PI/180.0)*2.0;
	
	pitchPosition.x -=  (rotationPitch->w/2)-(pitchImage->w/2);
    pitchPosition.y -=  (rotationPitch->h/2)-(pitchImage->h/2);
	
	SDL_Rect cameraPitch;
	cameraPitch.x =  0;
	cameraPitch.y =  (int)pitch + 140.0;  // pitch offset
	cameraPitch.w =  109;
    cameraPitch.h =  77;
	
	/***************  SPEED SCALE IMAGE ****************/
	
	
	SDL_Rect speedPosition={41,35,0,0};
	
	SDL_Rect cameraSpeed;
	cameraSpeed.x =  0;
	if (speed >= 2) {
		y = 357 + (speed - 1)* -12;
	} else {
		y = 345;
	}
	
	if (speed > 30) y = 14;
	cameraSpeed.y =  (int)y;
	cameraSpeed.w =  26;
    cameraSpeed.h =  106;
	
	/***************  ALTITUDE SCALE IMAGE ****************/
	
	
	SDL_Rect altitudePosition={213,35,0,0};
	
	SDL_Rect cameraAltitude;
	cameraAltitude.x =  0;
	
	y = 345 + (altitude - 1)* -24;
	if (altitude < -2 ) {
		y = 417;
	}
	
	if (altitude > 15) y = 9;
	cameraAltitude.y =  (int)y;
	cameraAltitude.w =  32;
    cameraAltitude.h =  106;
	
		
	SDL_BlitSurface( rotationHorizon, &camera, screen, &horizonPosition);
	SDL_BlitSurface( rotationUpperCompass, NULL, screen, &upperCompassPosition);
	SDL_BlitSurface( rotationPitch,   &cameraPitch, screen, &pitchPosition);
	SDL_BlitSurface( speedImage     , &cameraSpeed, screen, &speedPosition);
	SDL_BlitSurface( altitudeImage  , &cameraAltitude, screen, &altitudePosition);
	SDL_BlitSurface( backDropImage  , NULL, screen, &backGroundPosition);
	SDL_BlitSurface( rotationCompass, NULL, screen, &compassPosition);
	SDL_BlitSurface( backGroundImage, NULL, screen, &backGroundPosition);
	SDL_BlitSurface( rotationNeedle , NULL, screen, &needlePosition);
	SDL_BlitSurface( planeImage     , NULL, screen, &planePosition);
	SDL_BlitSurface( iasImage       , NULL, screen, &iasPosition);
	
	char str[10];
	
	sprintf(str, "%03.0f", yaw);
	DrawText(str, 131, 119);
	
	if (speed < 10) {
		sprintf(str, "%3.1f", speed);
	}else{
		sprintf(str, "%3.0f", speed);
	}
	if (speed > 2) {
		DrawText(str, 47, 81);
	}else{
		DrawText("---", 47, 81);
	}
	DrawText(str, 47, 140);
	
	
	if (altitude > 0) {
		sprintf(str, "%4.0f", altitude*100);
		DrawText(str, 218, 81);
	} else {
		DrawText(" -00",218,81);
	}
	
	
	sprintf(str, "%3.0f", temperature*10);
	DrawText(str,13,218);
	
	//Send the surface to the display.
	SDL_Flip(screen);

	//Free surfaces.
    SDL_FreeSurface(screen);
    return 0;

}

int DrawText(char *text, int x, int y) {
	SDL_Color color={255,255,255};
	SDL_Surface *text_surface;
	

	if(!(text_surface = TTF_RenderText_Solid(text_font,text,color))) {
		//handle error here, perhaps print TTF_GetError at least
		printf("Unable to setvideo: %s\n", SDL_GetError());
        return 1;
		//printf("Error TTF\n");
	} else {
		SDL_Rect textPosition={x, y, 0, 0};
		SDL_BlitSurface(text_surface, NULL, screen, &textPosition);
		//perhaps we can reuse it, but I assume not for simplicity.
		SDL_FreeSurface(text_surface);
	}
	return 0;
}


int OnDraw(SDL_Surface* Surf_Dest, SDL_Surface* Surf_Src, int X, int Y, int X2, int Y2, int W, int H) {
    if(Surf_Dest == NULL || Surf_Src == NULL) {
        return 0;
    }
 
    SDL_Rect DestR;
 
    DestR.x = X;
    DestR.y = Y;
 
    SDL_Rect SrcR;
 
    SrcR.x = X2;
    SrcR.y = Y2;
    SrcR.w = W;
    SrcR.h = H;
 
    SDL_BlitSurface(Surf_Src, &SrcR, Surf_Dest, &DestR);
 
    return 1;
}


