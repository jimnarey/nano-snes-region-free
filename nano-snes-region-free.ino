
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
 *  | [ ]         5V                  GND         [ ] |
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
 *  R = red; G = green; B = blue; r = reset; c = CIC; p = PPU; L = latch; C = clock
 *  
 *  Documentation on whether the analog pins on the Nano can be used for digital input is inconsistent.
 *  This says no (see end): https://www.arduino.cc/reference/en/language/functions/digital-io/digitalread/
 *  This says yes: https://www.arduino.cc/en/Tutorial/Foundations/AnalogInputPins
 *  This says A6 & A7 can't at the HW level, so not even using port manipulation: https://arduino.stackexchange.com/questions/79303/unable-to-use-analog-pins-as-digital-inputs
 *  
 *  Helpful reference for reading a SNES controller: https://github.com/burks10/Arduino-SNES-Controller/blob/master/Arduino%20Micro%20SNES%20Controller/USBSNES/Arduino%20Micro%20SNES%20Controller.ino
 *  
 *
 *  LED blended colours: 
 *  - purple = 255, 0, 255
 *  - yellow = 255, 125, 0
 *
 */

// TODO - Why the need for a reset on start-up?

#define DEBUG

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

int resetPin = 8;

int latchPin = 3; // Normally low?
int clockPin = 2;
int serialPin = 5; 

int redPin = 9;
int greenPin = 10;
int bluePin = 11;

int PPUPin = 6;
int CICPin = 7;

int state60Hz = LOW;
int stateCIC = LOW;

volatile int latchState = 0;
volatile int cycleStart = 0;
volatile int clockState = 0;
volatile int cycleCounter = 0;

volatile int buttonsState = 0;

int const resetMask = 1540;
int const cicToggleMask = 1030;
int const refreshToggleMask = 1029;

void readLatch()
{

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

void setLedColour(int intensityRed, int intensityGreen, int intensityBlue) {
  analogWrite(redPin, intensityRed);
  analogWrite(greenPin, intensityGreen);
  analogWrite(bluePin, intensityBlue);
}

void flashLed(int intensityRed, int intensityGreen, int intensityBlue) {
  for (int i = 0; i < 3; i++) {
    setLedColour(intensityRed, intensityGreen, intensityBlue);
    delay(1000);
    setLedColour(0, 0, 0);
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

  Serial.begin(115200);
  Serial.println();

  pinMode(resetPin, INPUT);
  pinMode(latchPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(latchPin), readLatch, FALLING);
  pinMode(clockPin, INPUT);
  pinMode(serialPin, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  pinMode(PPUPin, OUTPUT);
  pinMode(CICPin, OUTPUT);
  digitalWrite(PPUPin, state60Hz);
  digitalWrite(CICPin, stateCIC);
  setLedColour(255, 0, 0);

}

void loop() {

  if (buttonsState == 1540) {
    resetSNES();
  } else if (buttonsState == 1030) {
     if (stateCIC == HIGH) {
      // Disable CIC -- is this right?
      stateCIC = LOW;
      // digitalWrite(CICPin, LOW);          
      flashLed(255, 0, 255);
    } else {
      // Enable CIC -- is this right?
      stateCIC = HIGH;
      // digitalWrite(CICPin, HIGH);
      flashLed(255, 125, 0);
    }   
  } else if (buttonsState == 1029) {
    if (state60Hz == HIGH) {
      // Disabe 60Hz -- is this right?
      state60Hz = LOW;
      // digitalWrite(PPUPin, LOW);
      flashLed(0, 255, 0);
    } else {
      // Enable 60Hz -- is this right? 
      state60Hz = HIGH;
      // digitalWrite(PPUPin, HIGH);
      flashLed(255, 0, 255);
    }
  }
  
#ifdef DEBUG
  Serial.println();
  Serial.println(buttonsState);
#endif
  delay(50);
  
}



