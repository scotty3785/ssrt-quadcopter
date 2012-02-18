#include "/home/scott/sketchbook/ArduMegaTest1/defines.h"


void setup() {                
  // initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards:
  pinMode(C_LED_PIN, OUTPUT);
  pinMode(B_LED_PIN, OUTPUT);
  pinMode(A_LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(C_LED_PIN, HIGH);   // set the LED on
  delay(100);              // wait for a second
  digitalWrite(C_LED_PIN, LOW);    // set the LED off
  delay(100);              // wait for a second
  digitalWrite(B_LED_PIN, HIGH);   // set the LED on
  delay(100);              // wait for a second
  digitalWrite(B_LED_PIN, LOW);    // set the LED off
  delay(100);              // wait for a second
  digitalWrite(A_LED_PIN, HIGH);   // set the LED on
  delay(100);              // wait for a second
  digitalWrite(A_LED_PIN, LOW);    // set the LED off
  delay(100);              // wait for a second
  
}
