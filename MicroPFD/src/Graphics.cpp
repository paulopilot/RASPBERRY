#include "Graphics.h"

using namespace std;

Graphics::Graphics(int video_source)
{
    _video_source = video_source;
}

Graphics::~Graphics()
{
    destroy();
    SDL_Quit();
}

void Graphics::destroy() {
    SDL_FreeSurface(rotationPitch);
    SDL_FreeSurface(rotationNeedle);
    SDL_FreeSurface(rotationUpperCompass);
    SDL_FreeSurface(rotationCompass);
    SDL_FreeSurface(rotationHorizon);
    SDL_FreeSurface(altitudeImage);
    SDL_FreeSurface(speedImage);
    SDL_FreeSurface(pitchImage);
    SDL_FreeSurface(iasImage);
    SDL_FreeSurface(planeImage);
    SDL_FreeSurface(needleImage);
    SDL_FreeSurface(upperCompassImage);
    SDL_FreeSurface(compassImage);
    SDL_FreeSurface(backDropImage);
    SDL_FreeSurface(backGroundImage);
    SDL_FreeSurface(horizonImage);
    SDL_FreeSurface(screen);
}

int Graphics::initSDL() {
	char *device = (char *)"SDL_FBDEV=/dev/fb1";
	
	//fb1 = small TFT.   fb0 = HDMI/RCA output
	//if (_video_source == 0)
	//	*device = (char *)"SDL_FBDEV=/dev/fb0";	// HDMI
	//if (_video_source == 1)
	//	*device = (char *)"SDL_FBDEV=/dev/fb1"; // TFT
	
	putenv(device);	// TFT

	//Initialize  SDL and disable mouse
    SDL_Init(SDL_INIT_VIDEO);
    SDL_ShowCursor(SDL_DISABLE);

	//Get information about the current video device.  E.g. resolution and bits per pixal
    videoInfo = SDL_GetVideoInfo ();

	//Setup a Video mode.
    screen = SDL_SetVideoMode(videoInfo->current_w, videoInfo->current_h, videoInfo->vfmt->BitsPerPixel, SDL_SWSURFACE );

    if ( screen == NULL ) {
        fprintf(stderr, "Unable to setvideo: %s\n", SDL_GetError());
        return 1;
    }

    TTF_Init();
    term_font = TTF_OpenFont("/usr/share/fonts/truetype/terminus/TerminusTTF-4.40.1.ttf", 12);
    if (term_font == NULL) {
		printf("\nCould not load font\n");
		return 1;
	}

    return 0;
}


SDL_Surface* Graphics::loadImage(const char *file) {

    //Load image.

    SDL_Surface* tempImage = IMG_Load(file);

    if (tempImage == NULL) {
        //cout << "Error loading " << image << " file.\n";
        printf("Error loading %s file.\n", file);
        SDL_Quit();
        return NULL;
    }
    //Convert the image to a format we can use.
    SDL_Surface* result = SDL_DisplayFormatAlpha(tempImage);
    SDL_FreeSurface(tempImage);
    return result;

    return NULL;
}

int Graphics::initScreen() {

	horizonImage = loadImage("images/horizon.png");
	//horizonImage = IMG_Load("horizon.png");
	if (horizonImage == NULL) return 1;

	backGroundImage = loadImage("images/background.png");
	if (backGroundImage == NULL) return 1;

	backDropImage =  loadImage("images/backdrop.png");
    if (backDropImage == NULL) return 1;

	//Load image.
	compassImage =  loadImage("images/compassrose.png");
    if (compassImage == NULL) return 1;

	//Load image.
	upperCompassImage =  loadImage("images/uppercompass.png");
    if (upperCompassImage == NULL) return 1;

	//Load image.
	needleImage =  loadImage("images/needle.png");
    if (needleImage == NULL) return 1;

	//Load image.
	planeImage =  loadImage("images/plane.png");
    if (planeImage == NULL) return 1;

	//Load image.
	iasImage =  loadImage("images/IAS_image.png");
    if (iasImage == NULL) return 1;

	//Load image.
	pitchImage =  loadImage("images/pitchImage.png");
    if (pitchImage == NULL) return 1;

	//Load image.
	speedImage =  loadImage("images/speed_scale.png");
    if (speedImage == NULL) return 1;

	//Load image.
	altitudeImage =  loadImage("images/altitude_scale.png");
    if (altitudeImage == NULL) return 1;


	//SDL_free(tempImage);

	backGroundPosition.x = (videoInfo->current_w/2)-(backGroundImage->w/2);
	backGroundPosition.y = (videoInfo->current_h/2)-(backGroundImage->h/2);

	return 0;
}


int Graphics::drawText(const char *text, Sint16 x, Sint16 y) {
	SDL_Color color={255,255,255};
	SDL_Surface *text_surface;


	if(!(text_surface = TTF_RenderText_Solid(term_font,text,color))) {
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

void Graphics::drawScreen(sensors_event_t *ahrs) {
    float speed = 0;
	float altitude = ahrs->baro.altitude;
	float temperature = ahrs->baro.temperature;
	float pitch = 0;//sensor->pitch;
	float heading = ahrs->orientation.heading;
	float roll = 0;//sensor->roll;
	float yaw = 0;//heading;

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

	SDL_Rect horizonPosition={(Sint16)x,(Sint16)y,0,0};

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

	SDL_Rect upperCompassPosition={(Sint16)x,(Sint16)y,0,0};

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

	SDL_Rect pitchPosition={(Sint16)x,(Sint16)y,0,0};

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

	float alt_tmp;

	alt_tmp = altitude / 100;

	y = 345 + (alt_tmp - 1)* -24;
	if (alt_tmp < -2 ) {
		y = 417;
	}

	if (alt_tmp > 15) y = 9;
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

	//printf("Yaw=%3.0f\n", yaw);

	sprintf(str, "%03d", (int)yaw);
	drawText(str, 131, 119);

	if (speed < 10) {
		sprintf(str, "%3.1f", speed);
	}else{
		sprintf(str, "%3.0f", speed);
	}
	if (speed > 2) {
		drawText(str, 47, 81);
	}else{
		drawText("---", 47, 81);
	}
	drawText(str, 47, 140);


	if (altitude > 0) {
		sprintf(str, "%4.0f", altitude);
		drawText(str, 218, 81);
	} else {
		drawText(" -00",218,81);
	}


	sprintf(str, "%3.0f", temperature*10);
	drawText(str,13,218);

	//Send the surface to the display.
	SDL_Flip(screen);

	//Free surfaces.
    SDL_FreeSurface(screen);
    //destroy();
}

