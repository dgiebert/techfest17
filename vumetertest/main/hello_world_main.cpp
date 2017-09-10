#include "Arduino.h"

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define RIGHT_PIN   15
#define LEFT_PIN   	21
#define NUMPIXELS      80

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel left_strip = Adafruit_NeoPixel(NUMPIXELS, LEFT_PIN, NEO_GRB + NEO_KHZ800);

/*
Bottom row:
A0 - this is an analog input A0 and also an analog output DAC2. It can also be used
as a GPIO #26
A1 - this is an analog input A1 and also an analog output DAC1. It can also be used
as a GPIO #25
A2 - this is an analog input A2 and also GPI #34. Note it isnot an output-capable pin!
A3 - this is an analog input A3 and also GPI #39. Note it isnot an output-capable pin!
A4 - this is an analog input A4 and also GPI #36. Note it isnot an output-capable pin!
A5 - this is an analog input A5 and also GPIO #4
21 - General purpose IO pin #21
Top row:
13 - This is GPIO #13 and also an analog input A12. It's also connected to the red
LED next to the USB port
12 - This is GPIO #12 and also an analog input A11. This pin has a pull-down resistor
built into it, we recommend using it as an output only, or making sure that the pulldown
is not affected during boot.
27 - This is GPIO #27 and also an analog input A10
33 - This is GPIO #33 and also an analog input A9. It can also be used to connect a
32 KHz crystal.
15 - This is GPIO #15 and also an analog input A8
32 - This is GPIO #32 and also an analog input A7. It can also be used to connect a 32 KHz crystal.
14 - This is GPIO #14 and also an analog input A6
*/

int left_in = 26;
int right_in = 25;
int sound;

const int level_size = 8;
int left_leds[level_size] = {12, 4, 21, 12};
int right_leds[level_size] = {32, 15, 33, 27};
int lled0 = 14;
int lled1 = 32;
int lled2 = 15;
int lled3 = 33;
int rled0 = 27;
int rled1 = 4;
int rled2 = 21;
int rled3 = 12;

// Peak signal value
double peak_signal = 0;


// Left level from 0 to level_size
int l_level;
int r_level;

const int window = 20;
int l_values[window] = {0, 0, 0, 0};
int r_values[window] = {0, 0, 0, 0};

double avgArr(int values[]) {
    int sum = 0;

    for (int i = 0; i < window; i++) {
        sum += values[i];
    }

    return ((double) sum) / window;
}

int computeLevel(int values[]) {
  // calculate percentage of max
  if (peak_signal == 0) return 0;

  double value = avgArr(values);

  double percentage = value / peak_signal;
  double level = percentage * level_size;
  return (int) level;
}

void computeLevels() {
  l_level = computeLevel(l_values);
  r_level = computeLevel(r_values);
}

void displayLevels() {
  	int l = 0;
  	int r = 0;

	/*
	  for (; l < l_level; l++)    digitalWrite(left_leds[l], HIGH);
	  for (; r < r_level; r++)    digitalWrite(right_leds[r], HIGH);
	  for (; l < level_size; l++) digitalWrite(left_leds[l], LOW);
	  for (; r < level_size; r++) digitalWrite(right_leds[r], LOW);
	*/

  	uint32_t lColor = left_strip.Color(0,150,0);
	uint32_t rColor = left_strip.Color(150,0,0);


	for (; l < level_size; l++) {
		if (l > l_level) {
			for (int i = 0; i < 5*8; i+=8) left_strip.setPixelColor(l+i, left_strip.Color(0,0,0));
		} else {
			for (int i = 0; i < 5*8; i+=8) left_strip.setPixelColor(l+i, lColor);
		}
	}

	for (; r < level_size; r++) {
		if (r > r_level) {
			for (int i = 0; i < 5*8; i+=8) left_strip.setPixelColor(r+i+40, left_strip.Color(0,0,0));
		} else {
			for (int i = 0; i < 5*8; i+=8) left_strip.setPixelColor(r+i+40, rColor);
		}
	}
	//right_strip.show(); // This sends the updated pixel color to the hardware.
	left_strip.show();
}

void updatemax() {
  int right_value = avgArr(r_values);
  int left_value = avgArr(l_values);
  peak_signal = max(right_value, peak_signal);
  peak_signal = max(left_value, peak_signal);
  //Serial.print("PEAK: "); Serial.println(peak_signal);
}

void setup() {
  pinMode(lled0, OUTPUT);
  pinMode(lled1, OUTPUT);
  pinMode(lled2, OUTPUT);
  pinMode(lled3, OUTPUT);
  pinMode(rled0, OUTPUT);
  pinMode(rled1, OUTPUT);
  pinMode(rled2, OUTPUT);
  pinMode(LEFT_PIN, OUTPUT);
  pinMode(RIGHT_PIN, OUTPUT);
  pinMode(rled3, OUTPUT);

  pinMode(left_in, INPUT);
  pinMode(right_in, INPUT);

  Serial.begin(115200);
  Serial.println("INIT");
  //right_strip.begin(); // This initializes the NeoPixel library.
  left_strip.begin();

}

void fillArray() {
    for (int i = 0; i < level_size; i++) {
        l_values[i] = analogRead(left_in);
        r_values[i] = analogRead(right_in);
        delay(1);
    }
}

void loop() {
  delay(10);

  fillArray();
  computeLevels();
  updatemax();
  //Serial.print("LEVEL L: "); Serial.println(l_level);
  //Serial.print("LEVEL R: "); Serial.println(r_level);
  displayLevels();
}
