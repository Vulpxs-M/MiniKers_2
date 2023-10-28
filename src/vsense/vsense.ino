#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // for Serial

int PORT_vsense    = A1;
int vin_raw = 0;
float mv_per_lsb = 3600.0F/1024.0F/0.129F; // 10-bit ADC with 3.6V input range + 0.129 V/unit

void setup() {
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
}

void loop() {
  vin_raw = analogRead(PORT_vsense);

  Serial.println(String(" [") + (float) vin_raw * mv_per_lsb + String(" mV]"));

  delay(100);
}
