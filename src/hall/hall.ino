#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // for Serial

int PORT_hall_u = 11;
int PORT_hall_v = 12;
int PORT_hall_w = 13;
int val_u = 0;
int val_v = 0;
int val_w = 0;

void setup() {
  pinMode(PORT_hall_u, INPUT);
  pinMode(PORT_hall_v, INPUT);
  pinMode(PORT_hall_w, INPUT);
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
}

void loop() {
  val_u = digitalRead(PORT_hall_u);
  val_v = digitalRead(PORT_hall_v);
  val_w = digitalRead(PORT_hall_w);
  Serial.println(val_u + String(" ") + val_v + String(" ") + val_w);
  delay(100);
}
