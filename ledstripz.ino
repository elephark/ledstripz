#include <SPI.h>
#include <FastLED.h>

#define NUM_LEDS 150
#define DATA_PIN 17

typedef enum mode {
  TestChase,
  AllSolid,
  TwinkleSolid,
  SleepDimmer,
  ColorWave,
  RainbowWave,
  DebugBits,
  // 
  modeCount
} Mode;

// variables

Mode curMode = TwinkleSolid;

const int led = LED_BUILTIN;
const int encoderCSPin = 10;
SPISettings settings_74HC165(2000000, MSBFIRST, SPI_MODE0);
CRGB leds[NUM_LEDS];
unsigned long led_delay_timer;
unsigned long encoder_delay_timer;
unsigned int led_delay = 100;         // in ms
unsigned int encoder_delay = 1;       // in ms

// for test chase mode
unsigned int testChaseCurLed = 0;

// for all solid mode
CHSV curSolidColor(180, 160, 30);

// for the encoders
uint8_t lastEncValue = 255;
uint8_t dummyValue = 0;
uint8_t *encParam[2] = { &curSolidColor.hue, &curSolidColor.val };
const uint8_t encStepSize = 4;


// function prototypes

void serviceLeds();
void serviceButtons();
void serviceEncoders();

void serviceTestChase();
void serviceAllSolid();
void serviceTwinkleSolid();
void serviceDebugBits();

// main setup function
void setup() {
  pinMode(led, OUTPUT);
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.clear(); // necessary? iunno

  pinMode(encoderCSPin, OUTPUT);
  SPI.begin();
  SPI.setSCK(14);

  led_delay_timer = millis();
  encoder_delay_timer = millis();
}

// main loop function
void loop() {
    if (millis() - encoder_delay_timer > encoder_delay) {
      encoder_delay_timer = millis();
      serviceEncoders();
    }
    if (millis() - led_delay_timer > led_delay) {
      led_delay_timer = millis();
      serviceLeds();
    }
  delay(1);
}



// auxiliary functions past this point


void serviceLeds() {
  switch(curMode) {
  case TestChase:
    serviceTestChase();
    break;
  case AllSolid:
    serviceAllSolid();
    break;
  case DebugBits:
    serviceDebugBits();
    break;
  case TwinkleSolid:
    serviceTwinkleSolid();
    break;
  case SleepDimmer:
  case ColorWave:
  case RainbowWave:
  default:
    break;
  }
}

void serviceButtons() {
  
}

void serviceEncoders() {  
  // first, read the encoders' raw values

  SPI.beginTransaction(settings_74HC165);
  // tap the SH/!LD line so the 74HC165 captures its inputs
  digitalWrite(encoderCSPin, LOW);
  digitalWrite(encoderCSPin, HIGH);

  lastEncValue = SPI.transfer(0xff);

  SPI.endTransaction();

  // the bits come in inverted
  lastEncValue = ~lastEncValue;

  // turn the gray code into consecutive integers
  lastEncValue ^= ((lastEncValue & 0x0a) >> 1);

  static uint8_t encoder_positions[2];
  uint8_t temp_encoders[2];
  temp_encoders[0] = lastEncValue & 0x03;
  temp_encoders[1] = (lastEncValue >> 2) & 0x03;

  // we're going to vote on a direction
  static int8_t encoder_votes[2];
  
  // no debouncing rn because I am a lazebones

  for (uint8_t i = 0; i < 2; i++) {
    // have we moved to the right?
    if (temp_encoders[i] == ((encoder_positions[i] + 1) & 0x03)) {
      encoder_positions[i] = temp_encoders[i];
      encoder_votes[i]++;
    }
    // have we moved to the left?
    else if (encoder_positions[i] == ((temp_encoders[i] + 1) & 0x03)) {
      encoder_positions[i] = temp_encoders[i];
      encoder_votes[i]--;
    }
    // have we warped to the other side?
    else if (temp_encoders[i] == ((encoder_positions[i] + 2) & 0x03)) {
      encoder_positions[i] = temp_encoders[i];
    }
  }
  // if we're in a detent, tally up any extant votes
  for (uint8_t i = 0; i < 2; i++) {
      if (!encoder_positions[i] && encoder_votes[i]) {
      if (encoder_votes[i] > 0) {
        encoder_votes[i] = 0;
        // make an adjustment upward
        if (*encParam[i] < 252) { *encParam[i] += encStepSize; }
      }
      else if (encoder_votes[i] < 0) {
        encoder_votes[i] = 0;
        // make an adjustment downward
        if (*encParam[i] > 3) { *encParam[i] -= encStepSize; }
      }
    }
  }
}


void serviceTestChase() {
  FastLED.clear();
  leds[testChaseCurLed] = CRGB::HotPink;
  leds[testChaseCurLed + 1] = CRGB::Purple;
  FastLED.show();
  
  // wrap earlier, since we're setting multiple leds
  testChaseCurLed++;
  if (testChaseCurLed >= NUM_LEDS - 1) { testChaseCurLed = 0; }
}


void serviceAllSolid() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = curSolidColor;
  }
  FastLED.show();
}

// debug lol?
static uint8_t wavePos[8] = { 1, 4, 7, 3, 0, 5, 2, 6 };
static uint8_t waveDir[8] = { 0, 1, 0, 1, 0, 1, 0, 1 };

// twinkle twinkle little aaaaaaaaa
void serviceTwinkleSolid() {
  FastLED.clear();
  // bounds check
  uint16_t targetTwinkleDepth = 40;
  uint8_t twinkleDepth = targetTwinkleDepth;
  if (curSolidColor.val + targetTwinkleDepth > 255) { twinkleDepth = 255 - curSolidColor.val; }
  uint8_t twinkleDepthStep = twinkleDepth / 8;

  // super janky timer ay
  // 1 4 7 3 8 5 2 6
  const uint8_t firingOrder[8] = { 1, 4, 7, 3, 0, 5, 2, 6 };
//  static uint8_t wavePos[8]; // debug lol?
//  wavePos = (wavePos + 1) & 0x07;
  for (uint8_t i = 0; i < 8; i++) {
    if (!waveDir[i]) {
      wavePos[i]++;
      if (wavePos[i] >= 7) {
        waveDir[i] = 1;
      }
    }
    else { // waveDir != 0
      wavePos[i]--;
      if (!wavePos[i]) {
        waveDir[i] = 0;
      }
    }
  }
  
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = curSolidColor;
    CHSV tmpColor = curSolidColor;
    tmpColor.val += ((wavePos[i & 0x07] & 0x07) * twinkleDepthStep);
    tmpColor.hue += ((wavePos[i & 0x07] & 0x07) * twinkleDepthStep) & 0xff;
//    tmpColor.sat -= (((wavePos[i & 0x07] + firingOrder[i & 0x07]) & 0x07) * twinkleDepthStep / 2) & 0xff;
    leds[i] = CHSV(tmpColor);
//      leds[i].setHSV(tmpColor);
  }


  // debug: display a value
//    for (int i = 0; i < 8; i++) {
//    if (curSolidColor.hue & (1 << i)) {
//      leds[i + 8] = CHSV(curSolidColor);
//    }
//  }

  
  FastLED.show();
}


void serviceDebugBits() {
  FastLED.clear();
  for (int i = 0; i < 8; i++) {
    if (curSolidColor.val & (1 << i)) {
      leds[i] = curSolidColor;
    }
  }
  FastLED.show();
}
