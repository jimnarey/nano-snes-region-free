
/* Arduino Nano:
 *
 *                        +-----+
 *  +---------------------| USB |---------------------+
 *  |                     +-----+                     |
 *  | [ ]     PB5 D13/SCK        MISO/D12 PB4     [ ] |
 *  | [ ]         3.3V           MOSI/D11 PB3 PWM [B] |
 *  | [ ]         V.ref     ___    SS/D10 PB2 PWM [G] |
 *  | [ ]     PC0 A0       / N \       D9 PB1 PWM [R] |
 *  | [ ]     PC1 A1      /  A  \      D8 PB0     [r] |
 *  | [ ]     PC2 A2      \  N  /      D7 PD7     [c] |
 *  | [ ]     PC3 A3       \_0_/       D6 PD6 PWM [p] | [via 2.2K resistor]
 *  | [ ]     PC4 A4/SDA               D5 PD5 PWM [S] |
 *  | [ ]     PC5 A5/SCL               D4 PD4     [ ] |
 *  | [ ]         A6              INT1/D3 PD3 PWM [L] |
 *  | [ ]         A7              INT0/D2 PD2     [C] |
 *  | [+]         5V                  GND         [-] |
 *  | [ ]         RST                 RST PC6     [ ] |
 *  | [ ]         GND   5V MOSI GND   RX  PD0     [ ] |
 *  | [ ]         Vin   [ ] [ ] [ ]   TX  PD1     [ ] |
 *  |                   [ ] [ ] [ ]                   |
 *  |                   MISO SCK RST                  |
 *  | NANO-V3                                         |
 *  +-------------------------------------------------+
 *  B6 & B7 mapped to crystal
 *  C6 & C7 not available
 *
 *  R = red; G = green; B = blue; r = reset; c = CIC; p = PPU; L = latch; C = clock; S = serial/data, + = +5V, - = 0V
 *  
 *  Documentation on whether the analog pins on the Nano can be used for digital input is inconsistent.
 *  This says no (see end): https://www.arduino.cc/reference/en/language/functions/digital-io/digitalread/
 *  This says yes: https://www.arduino.cc/en/Tutorial/Foundations/AnalogInputPins
 *  This says A6 & A7 can't at the HW level, so not even using port manipulation: https://arduino.stackexchange.com/questions/79303/unable-to-use-analog-pins-as-digital-inputs
 *  
 *  Helpful reference for reading a SNES controller: https://github.com/burks10/Arduino-SNES-Controller/blob/master/Arduino%20Micro%20SNES%20Controller/USBSNES/Arduino%20Micro%20SNES%20Controller.ino
 *  
 *  Code for reading controller adapted from this project: https://github.com/Nold360/lazy_mans_snes_reset/blob/master/main.c
 *
 */

// TODO - Why the need for a reset on start-up?

#include <EEPROM.h>

// #define DEBUG
// #define ENABLE_CIC_SWITCH

#define address60HzState 0
#ifdef ENABLE_CIC_SWITCH
#define addressCICState 2
#endif

#define bIndex 0
#define yIndex 1
#define selectIndex 2
#define startIndex 3
#define upIndex 4
#define downIndex 5
#define leftIndex 6
#define rightIndex 7
#define aIndex 8
#define xIndex 9
#define lIndex 10
#define rIndex 11

#define resetCombo 1540  // Left, Select, X
#define cicToggleCombo 1030  // Left, Select, B
#define refreshToggleCombo 1029  // Left, Select, Y


int resetPin = 8;

int latchPin = 3;
int clockPin = 2;
int serialPin = 5; 

int redPin = 9;
int greenPin = 10;
int bluePin = 11;

int PPUPin = 6;
int CICPin = 7;

int state60Hz;
int stateCIC;

volatile int latchState = 0;
volatile int cycleStart = 0;
volatile int clockState = 0;
volatile int cycleCounter = 0;
volatile int buttonsState = 0;

struct rgbColour {
  int red;
  int green;
  int blue;
};

rgbColour flashColour;
rgbColour rgbOff = {0,0,0};
rgbColour rgbRed = {255, 0, 0};
rgbColour rgbGreen = {0, 255, 0};
rgbColour rgbBlue = {0, 0, 255};
rgbColour rgbWhite = {255, 255, 255};
rgbColour rgbPurple = {255, 0, 255};
// rgbColour rgbYellow = {255. 125, 0}; Currently unused


void readLatch() {

	uint8_t i;
  buttonsState = 0;

	for(i = 0; i < 12; i++)
	{
		uint8_t clk = (PIND & (1 << clockPin));
		while((PIND & (1 << clockPin)) == clk)
			_delay_us(1);

		if ((PIND & (1 << serialPin)) == 0)
			buttonsState |= (1 << i);

		clk = (PIND & (1 << clockPin));
		while((PIND & (1 << clockPin)) == clk)
			_delay_us(1);
	}

}

int getSavedPinState(int eepromAddress) {
  int value;
  EEPROM.get(eepromAddress, value);
  if (value > 0) { // Handle previously unwritten EEPROM address, which will be 255
    return 1;
  } 
  return 0;
}

void setPinState(int pin, int eepromAddress, int value) {
  digitalWrite(pin, value);
  EEPROM.put(eepromAddress, value);
}

void setLedColour(rgbColour colour) {
  analogWrite(redPin, colour.red);
  analogWrite(greenPin, colour.green);
  analogWrite(bluePin, colour.blue);
}

void flashLed(rgbColour colour) {
  for (int i = 0; i < 3; i++) {
    setLedColour(colour);
    delay(1000);
    setLedColour(rgbOff);
    delay(200);
  }
}

void resetSNES() {
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, LOW);
  delay(500);
  pinMode(resetPin, INPUT);
}

void setup() {

#ifdef DEBUG
  Serial.begin(115200);
  Serial.println();
#endif

  // Controller read pins
  pinMode(latchPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(latchPin), readLatch, FALLING);
  pinMode(clockPin, INPUT);
  pinMode(serialPin, INPUT);

  // LED pins  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // SNES control pins
  pinMode(PPUPin, OUTPUT);
  pinMode(resetPin, INPUT);
#ifdef ENABLE_CIC_SWITCH
  pinMode(CICPin, OUTPUT);
#endif

  state60Hz = getSavedPinState(address60HzState);
#ifdef ENABLE_CIC_SWITCH
  stateCIC = getSavedPinState(addressCICState);
#endif

  digitalWrite(PPUPin, state60Hz);
  digitalWrite(CICPin, stateCIC);
  setLedColour(rgbRed);

}

void loop() {

  if (buttonsState == resetCombo) {
    resetSNES();
    flashLed(rgbRed);
  }
 
  else if (buttonsState == refreshToggleCombo) {
    if (state60Hz == HIGH) {
      state60Hz = LOW; // Enable 60Hz
      setPinState(PPUPin, address60HzState, state60Hz);
      flashLed(rgbGreen);
    } else {
      state60Hz = HIGH;
      setPinState(PPUPin, address60HzState, state60Hz);
      flashLed(rgbWhite);
    }
  }
#ifdef ENABLE_CIC_SWITCH
  else if (buttonsState == cicToggleCombo) {
     if (stateCIC == HIGH) {
      stateCIC = LOW; // Disable CIC
      setPinState(CICPin, addressCICState, stateCIC);
      flashLed(rgbPurple);
    } else {
      stateCIC = HIGH;
      setPinState(CICPin, addressCICState, stateCIC);
      flashLed(rgbBlue);
    }

  }
#endif

#ifdef DEBUG
  Serial.println();
  Serial.println(buttonsState);
#endif
  setLedColour(rgbRed);
  delay(50);
  
}



