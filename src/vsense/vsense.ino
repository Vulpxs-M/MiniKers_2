#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // for Serial

int PORT_vsense    = A1;
int PORT_sw1       = 6;
int PORT_hvgate    = 21;

bool hvgate_on = 0;
int vin_raw = 0;
float mv_per_lsb = 3600.0F/1024.0F/0.128F; // 10-bit ADC with 3.6V input range + 0.128 V/unit

void setup() {
  pinMode(PORT_hvgate, OUTPUT);
  digitalWrite(PORT_hvgate, LOW);

  pinMode(PORT_sw1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PORT_sw1), buttonPress, FALLING);

  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
}

void loop() {
  vin_raw = analogRead(PORT_vsense);

  Serial.println(String(" [") + (float) vin_raw * mv_per_lsb + String(" mV]"));

  delay(100);
}

void buttonPress() {
  hvgate_on = !hvgate_on;
  digitalWrite(PORT_hvgate, hvgate_on);
}
