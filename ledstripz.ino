#include <SPI.h>                // The encoders come in via a 74HC165 i/o expander
#include <WS2812Serial.h>       // Clever non-blocking LED driver
#define USE_WS2812SERIAL        // Hook the previous and next drivers together
#include <FastLED.h>            // Nice ways of dealing with serial RGB LEDs
#include <Bounce2.h>            // Debouncing for the buttons
#include "SparkFunLIS3DH.h"     // LIS3DH 3-axis accelerometer
#include "Wire.h"               // I2C for the LIS3DH

#define NUM_LEDS 150

// Usable pins:
//   Teensy LC:   1, 4, 5, 24 (jumper to 17 to get 5V signal level)
//   Teensy 4.0:  1, 8, 14, 17, 20, 24, 29, 39
//   Teensy 4.1:  1, 8, 14, 17, 20, 24, 29, 35, 47, 53
#define DATA_PIN 1

// The lighting modes available to choose from.
typedef enum mode {
  // Test Chase mode:
  // A single LED chases from one end to the other. Makes a good basic test.
  // (Actually, we use two of them with different colors...it looks cooler.)
  TestChase,
  // All Solid mode:
  // All LEDs are assigned the same color, adjustable by the encoders.
  AllSolid,
  // Twinkle Solid mode:
  // A base color is assigned from which individual LEDs deviate slightly over time in hue and value.
  // This results in a pleasing twinkling effect.
  TwinkleSolid,
  // Twinkle Bump mode:
  // Like Twinkle Solid mode, but bump is enabled.
  // Not implemented (as such) yet.
  TwinkleBump,
  // Sleep Dimmer mode:
  // Gradually dims everything to black over a period of several minutes. Good nacht, sleep tacht!
  // Not implemented yet.
  SleepDimmer,

  // ColorWave,

  // RainbowWave,
  
  // Debug Bits mode:
  // Uses the LEDs to show a binary value. Useful for debugging in a different way than the console.
  DebugBits,

  modeCount
} Mode;

// Note: I realize that these button names and descriptions aren't currently accurate, but the
// idea is that they will be when I'm done using the buttons for various debugging tasks.
typedef enum buttonNames {
  btnPower,     // turns lights on/off
  btnMode,      // cycles through available modes
  // btnFrontWall, // en/disables lights on the front wall

  btnCount
} ButtonNames;


////////////////////////////////// Variables //////////////////////////////////


// In which mode do we start up?
Mode curMode = TwinkleSolid;

const int led = LED_BUILTIN;
const int encoderCSPin = 10;
SPISettings settings_74HC165(2000000, MSBFIRST, SPI_MODE0);
CRGB leds[NUM_LEDS];
uint32_t led_delay_timer;
uint32_t encoder_delay_timer;
const uint32_t led_delay = 100;         // in ms
const uint32_t encoder_delay = 1;       // in ms

// Current LED for Test Chase mode.
uint16_t testChaseCurLed = 0;

// Current solid color for All Solid mode, and the base color for Twinkle modes.
CHSV curSolidColor(180, 200, 60);

// for the encoders
uint8_t lastEncValue = 255;
uint8_t *encParam[2] = { &curSolidColor.hue, &curSolidColor.val };
const uint8_t encStepSize = 4;

// This makes serial printing work better...
String str;

// Bump: On some trigger, brightness bumps up by some degree.
// It gradually decays back to normal.
uint8_t bumpValue = 0;
uint8_t bumpDegree = 35;
uint8_t bumpDecay = 4;
uint32_t bump_delay_timer;
const uint32_t bump_delay = 100;        // in ms

// The actual (well, Arduino-aliased) pins assigned to each button.
const int buttonPins[btnCount] {
  8, // btnPower     (blue)
  9, // btnMode      (yellow)
  // 7, // btnFrontWall (green)
};

// We have some buttons.
Bounce* btns[btnCount];

// We have an accelerometer. "IMU" for short.
LIS3DH imu;

// Timer for updating the accelerometer.
uint32_t imu_delay_timer;
const uint32_t imu_delay = 30; // in ms

// Timer for how long it "cools down" after crossing the acceleration threshold.
uint32_t cooldownTimer;
const uint32_t cooldownTimer_thresh = 250; // in ms

// Ignore any acceleration below this amount. This is pretty close to the noise floor, at least
// for my system, and may need to be adjusted if a large truck happens to be idling outside.
const float aMagnitude_thresh = 0.073; // in g

// Blink timer for the onboard LED, which can be used for simple diagnostic purposes.
uint32_t ledTimer;
const uint32_t ledBlinkDuration = 10; // in ms
bool ledIsOn = false;


////////////////////////////////// Function Prototypes //////////////////////////////////


void serviceLeds();
void serviceButtons();
void serviceEncoders();
void serviceBump();
void serviceIMU();

void serviceTestChase();
void serviceAllSolid();
void serviceTwinkleSolid();
void serviceDebugBits();

void bump();


////////////////////////////////// Main Functions //////////////////////////////////


// Main setup function.
void setup() {
  Serial.begin(9600);
  Serial.println("ledstripz: Serial port is up.");

  pinMode(led, OUTPUT);
  // I can't tell you why it's BRG here and GRB using just WS2812B, but it is.
  FastLED.addLeds<WS2812SERIAL, DATA_PIN, BRG>(leds, NUM_LEDS);
  FastLED.clear();  // Start with a clean slate.

  pinMode(encoderCSPin, OUTPUT);
  SPI.begin();
  SPI.setSCK(14);

  for (uint8_t i = 0; i < btnCount; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
    btns[i] = new Bounce();
    btns[i]->attach(buttonPins[i], INPUT_PULLUP);
    btns[i]->interval(10);
  }

  imu.settings.accelSampleRate = 50;  // Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
  imu.settings.accelRange = 2;        // Max G force readable.  Can be: 2, 4, 8, 16
  imu.begin();
  imu.writeRegister(LIS3DH_CTRL_REG2, 0x08); // enable HPF

  led_delay_timer = millis();
  encoder_delay_timer = millis();
  bump_delay_timer = millis();
}

// Main loop function.
void loop() {
  serviceButtons();
  
  // If the onboard LED is in the middle of a blink, turn it off when it's time.
  if (ledIsOn && (millis() - ledTimer > ledBlinkDuration)) {
    digitalWrite(LED_BUILTIN, LOW);
    ledIsOn = false;
  }
  
  if (millis() - imu_delay_timer > imu_delay) {
    imu_delay_timer = millis();
    serviceIMU();
  }

  if (millis() - encoder_delay_timer > encoder_delay) {
    encoder_delay_timer = millis();
    serviceEncoders();
  }
  
  if (millis() - led_delay_timer > led_delay) {
    led_delay_timer = millis();
    uint32_t enc_debug_timer = micros();
    serviceLeds();
    static int16_t enc_debug_count = 0;
    if (enc_debug_count++ >= 100) {
      uint32_t enc_debug_timer_diff = micros() - enc_debug_timer;
      Serial.println(str + "serviceLeds() took " + enc_debug_timer_diff + " us");
      enc_debug_count = 0;
    }
  }
  
  if (millis() - bump_delay_timer > bump_delay) {
    bump_delay_timer = millis();
    serviceBump();
  }
}


////////////////////////////////// Auxiliary Functions //////////////////////////////////


// Directory function that passes control based on our mode.
void serviceLeds() {
  switch (curMode) {
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
  // case ColorWave:
  // case RainbowWave:
  default:
    break;
  }
}

// Read the buttons and figure out what to do if they've been pushed.
void serviceButtons() {
  for (uint8_t i = 0; i < btnCount; i++) {
    btns[i]->update();
  }
  if (btns[btnPower]->fell()) {
    bump();
  }
  if (btns[btnMode]->fell()) {
    curSolidColor.hue += 33;
  }
}

// Read the encoders and figure out what to do if they've moved.
void serviceEncoders() {
  // First, read the encoders' raw values.

  SPI.beginTransaction(settings_74HC165);
  // Tap the SH/!LD line so the 74HC165 captures its inputs.
  digitalWrite(encoderCSPin, LOW);
  digitalWrite(encoderCSPin, HIGH);

  lastEncValue = SPI.transfer(0xff);

  SPI.endTransaction();

  // The bits come in inverted.
  lastEncValue = ~lastEncValue;

  // Turn the gray code into consecutive integers.
  lastEncValue ^= ((lastEncValue & 0x0a) >> 1);

  static uint8_t encoder_positions[2];
  uint8_t temp_encoders[2];
  temp_encoders[0] = lastEncValue & 0x03;
  temp_encoders[1] = (lastEncValue >> 2) & 0x03;

  // We're going to vote on a direction.
  static int8_t encoder_votes[2];

  // no debouncing rn because I am a lazebones

  for (uint8_t i = 0; i < 2; i++) {
    // Have we moved to the right?
    if (temp_encoders[i] == ((encoder_positions[i] + 1) & 0x03)) {
      encoder_positions[i] = temp_encoders[i];
      encoder_votes[i]++;
    }
    // Have we moved to the left?
    else if (encoder_positions[i] == ((temp_encoders[i] + 1) & 0x03)) {
      encoder_positions[i] = temp_encoders[i];
      encoder_votes[i]--;
    }
    // Have we warped to the other side?
    else if (temp_encoders[i] == ((encoder_positions[i] + 2) & 0x03)) {
      encoder_positions[i] = temp_encoders[i];
    }
  }
  // If we're in a detent, tally up any extant votes.
  for (uint8_t i = 0; i < 2; i++) {
    if (!encoder_positions[i] && encoder_votes[i]) {
      if (encoder_votes[i] > 0) {
        encoder_votes[i] = 0;
        // Make an adjustment upward.
        if (*encParam[i] < 252) { *encParam[i] += encStepSize; }
      }
      else if (encoder_votes[i] < 0) {
        encoder_votes[i] = 0;
        // Make an adjustment downward.
        if (*encParam[i] > 3) { *encParam[i] -= encStepSize; }
      }
    }
  }
}


// Test Chase mode:
// A single LED chases from one end to the other. Makes a good basic test.
// (Actually, we use two of them with different colors...it looks cooler.)
void serviceTestChase() {
  FastLED.clear();
  leds[testChaseCurLed] = CRGB::HotPink;
  leds[testChaseCurLed + 1] = CRGB::Purple;
  FastLED.show();

  // Wrap earlier, since we're setting multiple leds.
  testChaseCurLed++;
  if (testChaseCurLed >= NUM_LEDS - 1) { testChaseCurLed = 0; }
}

// All Solid mode:
// All LEDs are assigned the same color, adjustable by the encoders.
void serviceAllSolid() {
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = curSolidColor;
  }
  FastLED.show();
}

// debug lol?
static uint8_t wavePos[8] = { 1, 4, 7, 3, 0, 5, 2, 6 };
static uint8_t waveDir[8] = { 0, 1, 0, 1, 0, 1, 0, 1 };

// Twinkle Solid mode:
// A base color is assigned from which individual LEDs deviate slightly over time in hue and value.
// This results in a pleasing twinkling effect.
void serviceTwinkleSolid() {
  // FastLED.clear(); // commenting this out saves ~85us and doesn't appear to hurt anything.
  // bounds check
  uint16_t targetTwinkleDepth = 40;
  uint8_t twinkleDepth = targetTwinkleDepth;
  if (curSolidColor.val + targetTwinkleDepth > 255) { twinkleDepth = 255 - curSolidColor.val; }
  uint8_t twinkleDepthStep = twinkleDepth / 8;

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

  // Populate each LED.
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    if (i < 8) {
      leds[i] = curSolidColor;
      CHSV tmpColor = curSolidColor;
      tmpColor.val += ((wavePos[i & 0x07] & 0x07) * twinkleDepthStep);
      tmpColor.hue += ((wavePos[i & 0x07] & 0x07) * twinkleDepthStep) & 0xff;

      // Bump calculations
      if ((uint16_t)tmpColor.val + bumpValue > 255) { tmpColor.val = 255; } // prevent overflow
      else { tmpColor.val += bumpValue; }
      if (bumpValue > tmpColor.sat) { tmpColor.sat = 0; }
      else { tmpColor.sat -= bumpValue; }

      leds[i] = CHSV(tmpColor);
    }
    else {
      // We did the calculations for the first group of 8, why do them again?
      // This saves ~730us compared to doing those calculations again 142 times.
      leds[i] = leds[i & 0x07];
    }
  }

  FastLED.show();
}

void serviceDebugBits() {
  FastLED.clear();
  for (uint8_t i = 0; i < 8; i++) {
    if (curSolidColor.val & (1 << i)) {
      leds[i] = curSolidColor;
    }
  }
  FastLED.show();
}


void serviceBump() {
  // Decrement by the decay magnitude. If it underflows, just go back to 0.
  bumpValue -= bumpDecay;
  if (bumpValue > bumpDegree) { bumpValue = 0; }
}

void bump() {
  bumpValue = bumpDegree;
}

// Read the accelerometer and try to figure out if something interesting has happened.
void serviceIMU() {
  float aX = imu.readFloatAccelX();
  float aY = imu.readFloatAccelY();
  float aZ = imu.readFloatAccelZ();
  float aMagnitude = std::sqrt((aX * aX) + (aY * aY) + (aZ * aZ));

  if ((aMagnitude > aMagnitude_thresh) && (millis() - cooldownTimer > cooldownTimer_thresh)) {
    Serial.print("\nAccelerometer:\n");
    Serial.print(" M = ");
    Serial.println(aMagnitude, 4);

    cooldownTimer = millis();

    // Start the LED blink.
    digitalWrite(LED_BUILTIN, HIGH);
    ledIsOn = true;
    ledTimer = millis();
    // Also trigger a bump.
    bump();
  }
}
