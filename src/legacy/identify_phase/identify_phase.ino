#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // for Serial

// Port Assignment
int PORT_in1 = 12;
int PORT_in2 = 25;
int PORT_in3 = 1;
int PORT_en1 = 26;
int PORT_en2 = 24;
int PORT_en3 = 0;
int PORT_nfault = 2;
int PORT_nreset = 5;

int PORT_hall_u = 9;
int PORT_hall_v = 10;
int PORT_hall_w = 11;

int PORT_vsense = A1;
int PORT_isense = A0;
int PORT_cur1 = A3;
int PORT_cur2 = A2;
int PORT_cur3 = A5;

int PORT_sw1 = 6;

// Values for Serial Print
int val_nfault = 0;

int val_u = 0;
int val_v = 0;
int val_w = 0;

int vin_raw = 0;
float mv_per_lsb = 3600.0F/1024.0F/0.128F; // 10-bit ADC with 3.6V input range + 0.128 V/unit
int iin_raw = 0;
float ma_adj_const = 3600.0F/1024.0F / 10.0F/0.051F;
float k = 0.05;
float ma_raw = 0;
float ma_filtered = 0;
int val_cur1 = 0;
int val_cur2 = 0;
int val_cur3 = 0;

// Configuration and Global Parameters
int duty_cycle = bit(7) - 1;
// bit(7) - 1 = 127 in 0-255 -> 50%; bit(6) - 1 = 63 -> 25%
int duty_max = bit(8) - 1;

int bldc_direction = 0;
int bldc_step = 0;

// New
int PORT_hvgate = 21;
int PORT_lvgate = 13;
int PORT_usersw = 7;
int PORT_trigger = A4;

bool hvgate_on = 0;

int state = 0;

void setup() {
  pinMode(PORT_en1, OUTPUT);
  pinMode(PORT_en2, OUTPUT);
  pinMode(PORT_en3, OUTPUT);
  pinMode(PORT_nreset, OUTPUT);
  pinMode(PORT_nfault, INPUT);

  digitalWrite(PORT_en1, LOW);
  digitalWrite(PORT_en2, LOW);
  digitalWrite(PORT_en3, LOW);
  digitalWrite(PORT_nreset, HIGH);

  pinMode(PORT_hall_u, INPUT);
  pinMode(PORT_hall_v, INPUT);
  pinMode(PORT_hall_w, INPUT);

  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  HwPWM0.addPin(PORT_in1);
  HwPWM0.addPin(PORT_in2);
  HwPWM0.addPin(PORT_in3);

  HwPWM0.begin();
  HwPWM0.setResolution(8);
  HwPWM0.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_8);

  pinMode(PORT_sw1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PORT_sw1), buttonPress, FALLING);

  // New
  pinMode(PORT_hvgate, OUTPUT);
  digitalWrite(PORT_hvgate, LOW);

  pinMode(PORT_usersw, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PORT_usersw), HVbuttonPress, FALLING);

  pinMode(PORT_lvgate, OUTPUT);
  digitalWrite(PORT_lvgate, HIGH);
}

void loop() {

  // val_nfault = digitalRead(PORT_nfault);
  val_u = digitalRead(PORT_hall_u);
  val_v = digitalRead(PORT_hall_v);
  val_w = digitalRead(PORT_hall_w);

  Serial.println(val_u + String(" ") + val_v + String(" ") + val_w);
  Serial.println(String("State: ") + state);
  // Serial.println(String("nfault: ") + val_nfault);

  // vin_raw = analogRead(PORT_vsense);
  // Serial.println(String(" [") + (float) vin_raw * mv_per_lsb + String(" mV]"));

  // iin_raw = analogRead(PORT_isense);
  // Serial.println(String(" {") + (float) iin_raw + String(" mA}"));
  
  // val_cur1 = analogRead(PORT_cur1);
  // Serial.println(String(" {") + (float) val_cur1 + String(" mA}"));
  // val_cur2 = analogRead(PORT_cur2);
  // Serial.println(String(" {") + (float) val_cur2 + String(" mA}"));
  // val_cur3 = analogRead(PORT_cur3);
  // Serial.println(String(" {") + (float) val_cur3 + String(" mA}"));  

  delay(10);
    // delay(10);
    // HwPWM0.writePin(PORT_in1, 0, false);
    // digitalWrite(PORT_en1, LOW);
    // HwPWM0.writePin(PORT_in2, 0, false);
    // digitalWrite(PORT_en2, LOW);
    // HwPWM0.writePin(PORT_in3, 0, false);
    // digitalWrite(PORT_en3, LOW);
  // }

}

void buttonPress() {
  // if ( state > 1 )
  //   state--;
  // else
  //   state = 7;
  if ( state < 6 )
    state++;
  else
    state = 0;

  switch(state) {
    case 1:
      HwPWM0.writePin(PORT_in1, 0, false);
      digitalWrite(PORT_en1, LOW);
      HwPWM0.writePin(PORT_in2, duty_cycle, false);
      digitalWrite(PORT_en2, HIGH);
      HwPWM0.writePin(PORT_in3, 0, false);
      digitalWrite(PORT_en3, HIGH);
      break;
    case 2:
      HwPWM0.writePin(PORT_in1, duty_cycle, false);
      digitalWrite(PORT_en1, HIGH);
      HwPWM0.writePin(PORT_in2, 0, false);
      digitalWrite(PORT_en2, LOW);
      HwPWM0.writePin(PORT_in3, 0, false);
      digitalWrite(PORT_en3, HIGH);
      break;
    case 3:
      HwPWM0.writePin(PORT_in1, duty_cycle, false);
      digitalWrite(PORT_en1, HIGH);
      HwPWM0.writePin(PORT_in2, 0, false);
      digitalWrite(PORT_en2, HIGH);
      HwPWM0.writePin(PORT_in3, 0, false);
      digitalWrite(PORT_en3, LOW);
      break;
    case 4:
      HwPWM0.writePin(PORT_in1, 0, false);
      digitalWrite(PORT_en1, LOW);
      HwPWM0.writePin(PORT_in2, 0, false);
      digitalWrite(PORT_en2, HIGH);
      HwPWM0.writePin(PORT_in3, duty_cycle, false);
      digitalWrite(PORT_en3, HIGH);
      break;
    case 5:
      HwPWM0.writePin(PORT_in1, 0, false);
      digitalWrite(PORT_en1, HIGH);
      HwPWM0.writePin(PORT_in2, 0, false);
      digitalWrite(PORT_en2, LOW);
      HwPWM0.writePin(PORT_in3, duty_cycle, false);
      digitalWrite(PORT_en3, HIGH);
      break;
    case 6:
      HwPWM0.writePin(PORT_in1, 0, false);
      digitalWrite(PORT_en1, HIGH);
      HwPWM0.writePin(PORT_in2, duty_cycle, false);
      digitalWrite(PORT_en2, HIGH);
      HwPWM0.writePin(PORT_in3, 0, false);
      digitalWrite(PORT_en3, LOW);
      break;
    default:
      HwPWM0.writePin(PORT_in1, 0, false);
      digitalWrite(PORT_en1, LOW);
      HwPWM0.writePin(PORT_in2, 0, false);
      digitalWrite(PORT_en2, LOW);
      HwPWM0.writePin(PORT_in3, 0, false);
      digitalWrite(PORT_en3, LOW);
  }
}

// New
void HVbuttonPress() {
  if (hvgate_on)
    turnOffHV();
  else
    turnOnHV();
}

void turnOnHV() {
  hvgate_on = true;
  digitalWrite(PORT_hvgate, hvgate_on);
}

void turnOffHV() {
  hvgate_on = false;
  digitalWrite(PORT_hvgate, hvgate_on);
}
