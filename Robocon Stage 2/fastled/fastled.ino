#include <FastLED.h>
CRGB leds[26];
void setup() {
  // put your setup code here, to run once:
  FastLED.addLeds<WS2811, 6, GRB>(leds, 26).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(255);

}

void loop() {
  // put your main code here, to run repeatedly:
  fill_solid(leds, 26, CRGB::Red);
  FastLED.show();
  delay(3000);
  fill_solid(leds, 26, CRGB::Blue);
  FastLED.show();
  delay(3000);
  fill_solid(leds, 26, CRGB::Green);
  FastLED.show();
  delay(3000);
}
