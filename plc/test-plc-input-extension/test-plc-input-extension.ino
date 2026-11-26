#include <Controllino.h>  /* Usage of CONTROLLINO library allows you to use CONTROLLINO_xx aliases in your sketch. */

/*
  Use Controllino digital output status LEDs to test external input module 4 x inputs.


  IMPORTANT INFORMATION!
  Please, select proper target board in Tools->Board->Controllino MINI/MAXI/MEGA before Upload to your CONTROLLINO.
  (Please, refer to https://github.com/CONTROLLINO-PLC/CONTROLLINO_Library if you do not see the CONTROLLINOs in the Arduino IDE menu Tools->Board.)

  This example sketch does not require any additional parts, equipment, wires or power supply. Just your CONTROLLINO, USB cable (type A-B) and a PC.
 
*/

// the setup function runs once when you press reset (CONTROLLINO RST button) or connect the CONTROLLINO to the PC
void setup() {
  pinMode(CONTROLLINO_PIN_HEADER_DIGITAL_IN_10, INPUT);
  pinMode(CONTROLLINO_PIN_HEADER_DIGITAL_IN_11, INPUT);
  pinMode(CONTROLLINO_PIN_HEADER_DIGITAL_IN_12, INPUT);
  pinMode(CONTROLLINO_PIN_HEADER_DIGITAL_IN_13, INPUT);
  
  pinMode(CONTROLLINO_D0, OUTPUT);
  pinMode(CONTROLLINO_D1, OUTPUT);
  pinMode(CONTROLLINO_D2, OUTPUT);
  pinMode(CONTROLLINO_D3, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(CONTROLLINO_D0, digitalRead(CONTROLLINO_PIN_HEADER_DIGITAL_IN_10));
  digitalWrite(CONTROLLINO_D1, digitalRead(CONTROLLINO_PIN_HEADER_DIGITAL_IN_11));
  digitalWrite(CONTROLLINO_D2, digitalRead(CONTROLLINO_PIN_HEADER_DIGITAL_IN_12));
  digitalWrite(CONTROLLINO_D3, digitalRead(CONTROLLINO_PIN_HEADER_DIGITAL_IN_13));
  delay(10);
}
