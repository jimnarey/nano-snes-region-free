
/* Arduino Nano:
 *
 *                        +-----+
 *  +---------------------| USB |---------------------+
 *  |                     +-----+                     |
 *  | [ ]     PB5 D13/SCK        MISO/D12 PB4     [ ] |
 *  | [ ]         3.3V           MOSI/D11 PB3 PWM [B] |
 *  | [ ]         V.ref     ___    SS/D10 PB2 PWM [G] |
 *  | [ ]     PC0 A0       / N \       D9 PB1 PWM [R] |
 *  | [ ]     PC1 A1      /  A  \      D8 PB0     [ ] |
 *  | [ ]     PC2 A2      \  N  /      D7 PD7     [c] |
 *  | [ ]     PC3 A3       \_0_/       D6 PD6 PWM [p] |
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
 *  Documentation on whether the analog pins on the Nano can be used for digital input is inconsistent.
 *  This says no (see end): https://www.arduino.cc/reference/en/language/functions/digital-io/digitalread/
 *  This says yes: https://www.arduino.cc/en/Tutorial/Foundations/AnalogInputPins
 *  This says A6 & A7 can't at the HW level, so not even using port manipulation: https://arduino.stackexchange.com/questions/79303/unable-to-use-analog-pins-as-digital-inputs
 *  
 *  Helpful reference for reading a SNES controller: https://github.com/burks10/Arduino-SNES-Controller/blob/master/Arduino%20Micro%20SNES%20Controller/USBSNES/Arduino%20Micro%20SNES%20Controller.ino
 * 
 *  LED blended colours: 
 *  - purple = 255, 0, 255
 *  - yellow = 255, 125, 0
 *
 */

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


int latchPin = 3; // Normally low?
int clockPin = 2;
int serialPin = 5; 

int redPin = 9;
int greenPin = 10;
int bluePin = 11;

int PPUPin = 6;
int CICPin = 7;

int state60Hz = HIGH;
int stateCIC = LOW;

volatile int latchState = 0;
volatile int cycleStart = 0;
volatile int clockState = 0;
volatile int cycleCounter = 0;

volatile int buttonsState[12];


void readLatch() {

  latchState = (PIND & B00001000) >> 3;
  if (latchState == 0) {
    buttonsState[0] = (PIND & B00100000);
    cycleCounter = 1;
  }
}

void readClock() {
  clockState = (PIND & B00000100) >> 2;
  if (clockState == 1) {
    delayMicroseconds(5);
    buttonsState[cycleCounter] = (PIND & B00100000);
    cycleCounter++;
  }
  
}

void setLedColour(int intensityRed, int intensityGreen, int intensityBlue) {
  analogWrite(redPin, intensityRed);
  analogWrite(greenPin, intensityGreen);
  analogWrite(bluePin, intensityBlue);
}

void flashLed(int intensityRed, int intensityGreen, int intensityBlue) {
  for (i = 0; i < 3; ++i) {
    setLedColour(intensityRed, intensityGreen, intensityBlue);
    delay(300);
    setLedColour(0, 0, 0);
    delay(200);
  }
}

void setup() {

  Serial.begin(115200);
  pinMode(latchPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(latchPin), readLatch, CHANGE); // Change?
  pinMode(clockPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(clockPin), readClock, CHANGE);
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
  // Serial.println(buttonsState[leftIndex]);

  for (i = 0; i < 12; ++i) {
    if (buttonsState[i] != 0) {
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }

  if (buttonsState[selectIndex] != 0) {

    if buttonsState[leftIndex] != 0) {

      if (buttonsState[bIndex] != 0) {
        if (state60Hz == HIGH) {
          // Enable 50Hz
          state60Hz = LOW;
          digitalWrite(PPUPin, state60Hz);
          flashLed(0, 0, 255);
        }
      }

      else if (buttonsState[yIndex] != 0) {
        if (state60Hz == LOW) {
          // Enable 60Hz
          state60Hz = HIGH;
          digitalWrite(PPUPin, state60Hz);
          flashLed(0, 255, 0);
        }
      }

      else if (buttonsState[aIndex] != 0) {
        if (stateCIC == HIGH) {
          // Disable CIC
          stateCIC = LOW;
          digitalWrite(CICPin, stateCIC);          
          flashLed(255, 0, 255);
        }      
      }

      else if (buttonsState[xIndex] != 0) {
        if (state60Hz == LOW) {
          // Enable CIC
          stateCIC = HIGH;
          digitalWrite(CICPin, stateCIC);
          flashLed(255, 125, 0);
        }
      }

      else if (buttonsState[xIndex] != 0) {
        // Reset SNES
      }

      setLedColour(255, 0, 0);

    }

  }

  delay(100);

}



