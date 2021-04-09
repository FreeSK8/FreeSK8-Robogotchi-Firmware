#include <stdlib.h>

#include "nrf_delay.h"

#include "SSD1306.h"
#include "Adafruit_GFX.h"

#include "button_input.h"
#include "buzzer/nrf_pwm.h"



////////////////////////////////////////
// Game that needs it's own source file
////////////////////////////////////////

#if HAS_DISPLAY
// Dont ask
// frame counter
unsigned int frame = 0;
// scrore string buffer
char text[16];

// 'skate_push', 24x23px
const unsigned char skate_push[] =
{0x00, 0x00, 0x70, 0x00, 0x00, 0xf8, 0x00, 0x00, 0xf8, 0x00, 0x7c, 0xf8, 0x01, 0xff, 0x70, 0x07,
0xff, 0x00, 0x06, 0x3f, 0x00, 0x00, 0x7e, 0x00, 0x00, 0xfd, 0xe0, 0x01, 0xfb, 0xe0, 0x01, 0xf0,
0x00, 0x00, 0xf0, 0x00, 0x06, 0x78, 0x00, 0x7f, 0x3c, 0x00, 0xfe, 0x1c, 0x00, 0xfc, 0x1c, 0x00,
0x00, 0x1c, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x1c, 0x00, 0x0f, 0xff, 0xe0, 0x03, 0xff, 0x80, 0x02,
0x82, 0x80, 0x03, 0x03, 0x00};
// 'skate_push_2', 24x23px
const unsigned char skate_push2[] =
{0x00, 0x01, 0xc0, 0x00, 0x03, 0xe0, 0x00, 0x03, 0xe0, 0x00, 0x7f, 0xe0, 0x01, 0xff, 0xc0, 0x07,
0xff, 0x00, 0x06, 0x3f, 0x00, 0x00, 0x7e, 0x60, 0x00, 0xfd, 0xe0, 0x01, 0xf9, 0x80, 0x01, 0xf0,
0x00, 0x00, 0xf0, 0x00, 0x00, 0x78, 0x00, 0x01, 0xfc, 0x00, 0x01, 0xdc, 0x00, 0x01, 0xdc, 0x00,
0x01, 0xdc, 0x00, 0x01, 0xdc, 0x00, 0x03, 0x9c, 0x00, 0x0f, 0xff, 0xe0, 0x03, 0xff, 0x80, 0x02,
0x82, 0x80, 0x03, 0x03, 0x00};
// 'skate_jump', 24x23px
const unsigned char skate_jump[] =
{0x00, 0x70, 0x00, 0x00, 0x78, 0x00, 0x00, 0xf8, 0xc0, 0x00, 0x73, 0xc0, 0x00, 0x27, 0x00, 0x00,
0xfe, 0x00, 0x01, 0xf8, 0x00, 0x01, 0xf8, 0x00, 0x03, 0xf8, 0x00, 0x03, 0xf8, 0x00, 0x03, 0xf8,
0x00, 0x01, 0xfc, 0x00, 0x00, 0xfe, 0x00, 0x00, 0xc6, 0x00, 0x01, 0xc6, 0x00, 0x01, 0x86, 0x30,
0x01, 0x86, 0x60, 0x03, 0x87, 0xc0, 0x03, 0xfd, 0xc0, 0x1f, 0xc1, 0xc0, 0x03, 0x80, 0xc0, 0x02,
0x80, 0x00, 0x03, 0x80, 0x00};
// 'traffic_cone', 24x24px
const unsigned char traffic_cone[] =
{0x00, 0x20, 0x00, 0x00, 0x20, 0x00, 0x00, 0x70, 0x00, 0x00, 0x70, 0x00, 0x00, 0x70, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0x00, 0x00, 0xfc, 0x00, 0x01, 0xfc,
0x00, 0x01, 0xfc, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x02, 0x02, 0x00, 0x07, 0xff, 0x00, 0x07, 0xff, 0x40, 0x77, 0xff, 0x70, 0x3b, 0xfe, 0xe0, 0x04,
0x79, 0x80, 0x01, 0xfc, 0x00, 0x00, 0x70, 0x00};

// cloud_1 w:  24  h:  7
const unsigned char cloud_1[] =
{   0x00, 0xF0, 0x00, 0x73, 0x0C, 0x00, 0x88, 0x03,
    0x00, 0x81, 0x04, 0x80, 0x86, 0x00, 0x80, 0x79,
    0x83, 0x00, 0x00, 0x7C, 0x00 };

// skater_tumble, 32x20px
const unsigned char skater_tumble[] =
{0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x70, 0x00,
0x00, 0x00, 0x70, 0xe0, 0x00, 0x00, 0x71, 0xf0, 0x02, 0x00, 0x39, 0xf0, 0x07, 0xf8, 0x1d, 0xf0,
0x07, 0xfc, 0x1e, 0xe0, 0x00, 0xfe, 0x0f, 0x00, 0x00, 0x0e, 0x1f, 0x80, 0x00, 0x07, 0x3f, 0x80,
0x00, 0x37, 0xff, 0xc0, 0x00, 0xff, 0xfe, 0xe0, 0x01, 0xff, 0xfc, 0xe0, 0x03, 0xdf, 0xf8, 0xe0,
0x0f, 0x87, 0xf0, 0xe0, 0x0f, 0x01, 0xe0, 0xe0, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00};

// distance ran
int d, delta;
int cloud_1_y;
int d_jump, d_jump_t;
int d_run;
int d_tumble_t;
int ox;
// Really, don't ask
void play_game(){
	d = 0;
	delta = 0;
	cloud_1_y = 2;
	d_jump = 0;
	d_jump_t = 0;
	d_tumble_t = 0;
	d_run = 0;
	ox = 130;	

	while(true)
	{
		if (!d_run && isButtonPressed) {
			//TODO: why can't I do this?
			//while(isButtonPressed)
			//{
			//	nrf_delay_ms(100);
			//}
			d_run = 1;
		}

		if (d_tumble_t && isButtonPressed) {
			d = 0;
			delta = 0;
			cloud_1_y = 2;
			d_jump = 0;
			d_jump_t = 0;
			d_tumble_t = 0;
			d_run = 0;
			ox = 130;
			while(isButtonPressed)
			{
				nrf_delay_ms(100);
			}
			continue;
		}

		if (++frame>16000) frame = 0;

		// increase distance whilst running
		if (d_run && (++delta > 4)) {
			delta = 0; ++d; 
		}

		// obstacles
		if (d_run) {
			ox -= (frame%2)*(d/100) + 2;
			if (ox < -15) ox += 140 + rand() % 60;
		}

		// jump!
		if (!d_jump_t && isButtonPressed) {
			d_jump_t = 1;
			d_jump=5;

			beep_speaker_blocking(40, 10);

		} else if (d_jump_t) {
			++d_jump_t;

			if (d_jump_t<4) {
				d_jump +=4;
			} else if (d_jump_t<9) {
				d_jump +=2;
			} else if (d_jump_t<13) {
				d_jump +=1;
			} else if (d_jump_t == 16 || d_jump_t == 18) {
				d_jump +=1;
			} else if (d_jump_t == 20 || d_jump_t == 22) {
				d_jump -=1;
			} else if (d_jump_t>38) {
				d_jump = 0;
				d_jump_t = 0;
			} else if (d_jump_t>32) {
				d_jump -=4;
			} else if (d_jump_t>29) {
				d_jump -=2;
			} else if (d_jump_t>25) {
				d_jump -=1;
			}
		}

		// hit detect
		if (!d_tumble_t && ox > -10 && ox <10 && d_jump_t < 5) {
			d_tumble_t = 1;    
		}

		if (d_tumble_t) {
			if (d_tumble_t == 1) {
				beep_speaker_blocking(100,10);
			} else if (d_tumble_t == 15) {
				beep_speaker_blocking(200,10);
			} else if (d_tumble_t == 30) {
				beep_speaker_blocking(1500,10);
			}

			++d_tumble_t;
			if (d_jump > -4) {
				d_jump -= 1;
				ox -= 1;
			} else {
				d_run = 0;
			}
		}

		SSD1306_clearDisplay();

		// hud

		// score
		sprintf(text,"%d",d);
		Adafruit_GFX_print(text, 100, 0);
		
		// parallax clouds
		Adafruit_GFX_drawBitmap(128 -(d%128),cloud_1_y,cloud_1,24,7,WHITE);

		if (d%128 == 0) {
			cloud_1_y = rand() % 10;
		}

		// terrain
		if (d_jump > 4) {
			Adafruit_GFX_drawLine(0,30,128,30,WHITE);
		} else {
			Adafruit_GFX_drawLine(0,30,3,30,WHITE);
			Adafruit_GFX_drawLine(12,30,127,30,WHITE); 
		}

		// obstacles
		Adafruit_GFX_drawBitmap(ox,8,traffic_cone,24,24,WHITE);


		// dino
		int dy = 9-d_jump;

		// tumbles!
		if (d_tumble_t) {
			Adafruit_GFX_drawBitmap(0,dy,skater_tumble,32,20,WHITE);

		// runs!
		} else {
			//Adafruit_GFX_drawBitmap(0,dy,dino_top,24,18,WHITE);

			// Run, Dino, Run!
			if (d_run && !d_jump) {
				if ((frame%8)/4) {
					Adafruit_GFX_drawBitmap(0,dy,skate_push,24,23,WHITE);
				} else {
					Adafruit_GFX_drawBitmap(0,dy,skate_push2,24,23,WHITE);
				}
			} else if (d_run && d_jump) {
				Adafruit_GFX_drawBitmap(0,dy,skate_jump,24,23,WHITE);
			} else {
				Adafruit_GFX_drawBitmap(0,dy,skate_push2,24,23,WHITE);
			}
		}

		SSD1306_display(); //Game is allowed to update display when it wants
		nrf_delay_ms(16);
	}
}
//You didn't listen when I said don't ask, did you?
#endif