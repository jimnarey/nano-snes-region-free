
/* Arduino Nano:
 *
 *                        +-----+
 *  +---------------------| USB |---------------------+
 *  |                     +-----+                     |
 *  | [ ]     PB5 D13/SCK        MISO/D12 PB4     [ ] |
 *  | [ ]         3.3V           MOSI/D11 PB3 PWM [ ] |
 *  | [ ]         V.ref     ___    SS/D10 PB2 PWM [ ] |
 *  | [ ]     PC0 A0       / N \       D9 PB1 PWM [ ] |
 *  | [ ]     PC1 A1      /  A  \      D8 PB0     [ ] |
 *  | [ ]     PC2 A2      \  N  /      D7 PD7     [ ] |
 *  | [ ]     PC3 A3       \_0_/       D6 PD6 PWM [ ] |
 *  | [ ]     PC4 A4/SDA               D5 PD5 PWM [S] |
 *  | [ ]     PC5 A5/SCL               D4 PD4     [C] |
 *  | [ ]         A6              INT1/D3 PD3 PWM [L] |
 *  | [ ]         A7              INT0/D2 PD2     [ ] |
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
 */




int latchPin = 3; // Normally low?
int clockPin = 2;
int serialPin = 5; 

int redPin = 9;
int greenPin = 10;
int bluePin = 11;

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

void setup() {

  Serial.begin(115200);

  pinMode(latchPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(latchPin), readLatch, CHANGE); // Change?
  pinMode(clockPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(clockPin), readClock, CHANGE);
  pinMode(serialPin, INPUT);
  
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);


}

void loop() {
  Serial.println(buttonsState[11]);

}
