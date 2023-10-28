#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // for Serial

int PORT_ledr = 2;
int PORT_ledg = 16;
int PORT_ledb = 21;

void setup() {
  pinMode(PORT_ledr, OUTPUT);
  pinMode(PORT_ledg, OUTPUT);
  pinMode(PORT_ledb, OUTPUT);
}

void loop() {
  digitalWrite(PORT_ledr, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(PORT_ledr, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second

  digitalWrite(PORT_ledg, HIGH);
  delay(1000);
  digitalWrite(PORT_ledg, LOW);
  delay(1000);

  digitalWrite(PORT_ledb, HIGH);
  delay(1000);
  digitalWrite(PORT_ledb, LOW);
  delay(1000);
}