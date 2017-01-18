#ifndef GRAPHICS_H
#define GRAPHICS_H

#include <SDL/SDL.h>
#include <SDL/SDL_image.h>
#include <SDL/SDL_ttf.h>
#include <SDL/SDL_rotozoom.h>

#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>

#include "sensor.h"


class Graphics
{
    public:
        Graphics(
            int = 1
        );
        int initSDL();
        int initScreen();
        void drawScreen(sensors_event_t *ahrs);
        virtual ~Graphics();

    protected:
        int _video_source;

        SDL_Surface *screen = NULL;
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

        TTF_Font *term_font;// =  TTF_OpenFont("/usr/share/fonts/truetype/freefont/FreeSans.ttf", 10);
        const SDL_VideoInfo* videoInfo;
        SDL_Rect backGroundPosition;

        SDL_Surface* loadImage(const char *file);
        int drawText(const char *text, Sint16 x, Sint16 y);
        void destroy();
    private:

};

#endif // GRAPHICS_H
