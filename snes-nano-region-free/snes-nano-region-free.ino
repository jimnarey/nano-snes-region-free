
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
 *  | [ ]     PC3 A3       \_0_/       D6 PD6     [ ] |
 *  | [ ]     PC4 A4/SDA               D5 PD5 PWM [ ] |
 *  | [ ]     PC5 A5/SCL               D4 PD4 PWM [ ] |
 *  | [ ]         A6              INT1/D3 PD3 PWM [ ] |
 *  | [ ]         A7              INT0/D2 PD2     [ ] |
 *  | [ ]         5V                  GND         [ ] |
 *  | [ ]         RST                 RST PC6     [ ] |
 *  | [ ]         GND   5V MOSI GND   RX0 PD0     [ ] |
 *  | [ ]         Vin   [ ] [ ] [ ]   TX1 PD1     [ ] |
 *  |                   [ ] [ ] [ ]                   |
 *  |                   MISO SCK RST                  |
 *  | NANO-V3                                         |
 *  +-------------------------------------------------+
 *  B6 & B7 mapped to crystal
 *  C6 & C7 not available 
 *
 */

int latchPin = 0
int clockPin = 0

void setup() {
  pinMode(latchPin, INPUT);


}

void loop() {
  // put your main code here, to run repeatedly:

}
