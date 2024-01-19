#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // for Serial
#include <Adafruit_NeoPixel.h> // for NeoPixel LED

#define CW    1
#define CCW  -1

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

float k = 0.05; // filter constant
int prevSerial = 0;

int vin_raw = 0;
float vin_calculated = 0;
float vin_filtered = 0;

int iin_raw = 0;
float iin_calculated = 0;
float iin_filtered = 0;

int val_cur1 = 0;
int val_cur2 = 0;
int val_cur3 = 0;

// Configuration and Global Parameters
int duty_cycle = 150;
int duty_cycle_regen = 140;
// bit(7) - 1 = 127 in 0-255 -> 50%; bit(6) - 1 = 63 -> 25%
int duty_max = bit(8) - 1;

int bldc_direction = 0;
int bldc_step = 0;

// For RPM and Pulse Count
int direct = CW;
int pulseCount;
int openingAnglePulse = 1800;

float startTime;
float prevTime;
float pulseTimeW;
float pulseTimeU;
float pulseTimeV;
float avgPulseTime;

float PPM;
float RPM;

int holdingTime;

#define IDLE        0
#define RUNNING     1
#define HOLD        2
#define HARVESTING  3
int door_state = IDLE;


// New
int PORT_hvgate = 21;
int PORT_lvgate = 13;
int PORT_usersw = 7;
int PORT_trigger = A4;

bool hvgate_on = 0;

int PORT_led = 8;
Adafruit_NeoPixel onePixel(1, PORT_led, NEO_GRB + NEO_KHZ800);

// Setup
void setup() {
  // Digital Pins
  pinMode(PORT_in1, OUTPUT);
  pinMode(PORT_in2, OUTPUT);
  pinMode(PORT_in3, OUTPUT);
  pinMode(PORT_nreset, OUTPUT);
  pinMode(PORT_nfault, INPUT);

  digitalWrite(PORT_in1, LOW);
  digitalWrite(PORT_in2, LOW);
  digitalWrite(PORT_in3, LOW);
  digitalWrite(PORT_nreset, HIGH);

  pinMode(PORT_hall_u, INPUT);
  pinMode(PORT_hall_v, INPUT);
  pinMode(PORT_hall_w, INPUT);

  // For nrf52840 with native usb
  Serial.begin(115200);
  while ( !Serial ) delay(10);

  // PWM Configuration
  HwPWM0.addPin(PORT_en1);
  HwPWM0.addPin(PORT_en2);
  HwPWM0.addPin(PORT_en3);

  HwPWM0.begin();
  HwPWM0.setResolution(8);
  HwPWM0.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_2);

  // Initial State Detection
  val_u = digitalRead(PORT_hall_u);
  val_v = digitalRead(PORT_hall_v);
  val_w = digitalRead(PORT_hall_w);
  bldc_step = (val_u << 2) + (val_v << 1) + val_w;

  attachInterrupt(digitalPinToInterrupt(PORT_hall_u), hall_drive_u, CHANGE);      
  attachInterrupt(digitalPinToInterrupt(PORT_hall_v), hall_drive_v, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PORT_hall_w), hall_drive_w, CHANGE);

  pinMode(PORT_sw1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PORT_sw1), buttonPress, FALLING);


  // New
  pinMode(PORT_hvgate, OUTPUT);
  digitalWrite(PORT_hvgate, LOW);

  pinMode(PORT_usersw, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PORT_usersw), HVbuttonPress, FALLING);

  pinMode(PORT_lvgate, OUTPUT);
  digitalWrite(PORT_lvgate, HIGH);

  onePixel.begin();
  onePixel.clear();
  onePixel.setBrightness(20);
  onePixel.show();
}

void loop() {
  if ( door_state == HARVESTING ) {
    if ( RPM > 1200 ) {
      duty_cycle_regen = 150;
      bldc_regen();
    }
    else if ( RPM > 600 ) {
      duty_cycle_regen = (int) (-0.08 * RPM) + 250;
      bldc_regen();
    }
    else if ( RPM > 400 ) {
      duty_cycle_regen = 200;
      bldc_regen();
    }
    else {
      duty_cycle_regen = 0;
      bldc_idle();
      door_state = IDLE;
    }
  }

  if ((millis() - prevTime) > 200) {
    RPM = 0;
  } 

  if ( door_state == RUNNING && pulseCount > openingAnglePulse ) {
    bldc_direction = 0;
    door_state = HOLD;
    holdingTime = millis();
    bldc_brake();
  }

  if ( door_state == HOLD ) {
    if ( millis() - holdingTime >= 5000 ) {
      bldc_idle();
      door_state = IDLE;
    }
  }

  if ( door_state == IDLE && RPM > 600 ) {
    bldc_regen();
    door_state = HARVESTING;
  }

  vin_raw = analogRead(PORT_vsense);
  vin_calculated = vin_raw * 0.0272 + 0.0735;
  // vin_filtered = vin_calculated * k + (1.0 - k) * vin_filtered;

  iin_raw = analogRead(PORT_isense);
  // iin_calculated = (411.6 - (vin_calculated - 10) * 1.72 - iin_raw) / 145.3;
  // iin_filtered = iin_calculated * k + (1.0 - k) * iin_filtered;

  val_cur1 = analogRead(PORT_cur1);
  val_cur2 = analogRead(PORT_cur2);
  val_cur3 = analogRead(PORT_cur3);

  if ( millis() > prevSerial && hvgate_on && vin_calculated > 5 ) {
    // Serial.println(vin_raw);
    // Serial.println(iin_raw);
    // Serial.println(String(" [") + vin_filtered + String(" V]"));
    // Serial.println(String(" {") + iin_filtered + String(" A}"));
    // Serial.println(millis());

    // Serial.println(millis() + String(",") + vin_calculated + String(",") + iin_calculated + String(",") + door_state);
    Serial.println(millis() + String(",") + vin_raw + String(",") + iin_raw + String(",") + door_state + String(",") + RPM + String(",") + 
                    val_cur1 + String(",") + val_cur2 + String(",") + val_cur3 + String(",") + pulseCount);

    prevSerial = millis();
  }

  // Protection
  // if ( hvgate_on && door_state (iin_filtered > 2 || iin_filtered < -2) ) {
  //   turnOffHV();
  //   bldc_idle();
  //   door_state = IDLE;
  // }

  // Serial.println(String("Hall Step: ") + bldc_step);
  // Serial.println(String("RPM: ") + RPM);
  // Serial.println(String("Pulse Count: ") + pulseCount);
  // Serial.println(String("Detected Direction: ") + direct);
  // Serial.println(String("Control Direction: ") + bldc_direction);
  // Serial.println(String("Status: ") + door_state);

  // New
  // Serial.println(String("HV Gate: ") + hvgate_on);

  /*
  // Hall Position Detection
  val_u = digitalRead(PORT_hall_u);
  val_v = digitalRead(PORT_hall_v);
  val_w = digitalRead(PORT_hall_w);
  // Updating BLDC Step
  bldc_step = (val_u << 2) + (val_v << 1) + val_w;
  // Hall Report
  Serial.println(val_u + String(" ") + val_v + String(" ") + val_w);
  
  // nfault Pin Detection
  val_nfault = digitalRead(PORT_nfault);
  // nfault Report
  Serial.println(String("nfault: ") + val_nfault);
  */
  /*
  // Terminal Voltage and High-end Current Sensing
  vin_raw = analogRead(PORT_vsense);
  iin_raw = analogRead(PORT_isense);
  // Voltage and Current Report
  Serial.println(String(" [") + (float) vin_raw * mv_per_lsb + String(" mV]"));
  Serial.println(String(" {") + (float) iin_raw + String(" mA}"));
  
  // Per-Phase Current Sensing
  val_cur1 = analogRead(PORT_cur1);
  val_cur2 = analogRead(PORT_cur2);
  val_cur3 = analogRead(PORT_cur3);
  // Per-Phase Current Report
  Serial.println(String(" {") + (float) val_cur1 + String(" mA}"));
  Serial.println(String(" {") + (float) val_cur2 + String(" mA}"));
  Serial.println(String(" {") + (float) val_cur3 + String(" mA}"));  
  */
  // Delay for Detection and Sensing
  // delay(100);


  // New
  /*
  int trigger_out = analogRead(PORT_trigger);
  Serial.println(String("Trigger: ") + trigger_out);
  */

}

/**
 * Interrupt Functions
 */
void buttonPress() {
  if ( door_state == IDLE && pulseCount < openingAnglePulse ) {
    bldc_direction = CW;
    door_state = RUNNING;

    onePixel.setPixelColor(0, 255, 0, 0);
    onePixel.show();

    bldc_move();
  }
  else {
    bldc_idle();
    door_state = IDLE;
  }
}

void hall_drive_w() {
  startTime = millis();
  hall_read();
  direct = (val_w == val_v) ? CW : CCW;
  pulseTimeW = startTime - prevTime;
  calculate_rpm();
  if ( door_state == RUNNING )
    bldc_move();
}

void hall_drive_v() {
  startTime = millis();
  hall_read();
  direct = (val_v == val_u) ? CW : CCW;
  pulseTimeV = startTime - prevTime;
  calculate_rpm();
  if ( door_state == RUNNING )
    bldc_move();
}

void hall_drive_u() {
  startTime = millis();
  hall_read();
  direct = (val_u == val_w) ? CW : CCW;
  pulseTimeU = startTime - prevTime;
  calculate_rpm();
  if ( door_state == RUNNING )
    bldc_move();
}

/**
 * Helper Functions
 */
void hall_read() {
  val_u = digitalRead(PORT_hall_u);
  val_v = digitalRead(PORT_hall_v);
  val_w = digitalRead(PORT_hall_w);
  bldc_step = (val_u << 2) + (val_v << 1) + val_w;
}

void calculate_rpm() {
  pulseCount = pulseCount + direct;
  avgPulseTime = ((pulseTimeW + pulseTimeU + pulseTimeV)/3);
  PPM = (1000 / avgPulseTime) * 60;
  RPM = PPM / 36;
  prevTime = startTime;
}

/**
 * Motor Functions
 */
void bldc_move() {
  int step;

  // int ccw[] = {2, 4, 3, 6, 1, 5}; // works with directional sensing w/o pullup
  int ccw[] = {3, 5, 4, 1, 2, 6}; // most efficient
  // int cw[] = {1, 3, 2, 5, 6, 4}; // works with directional sensing w/o pullup
  int cw[] = {6, 2, 1, 4, 5, 3}; // most efficient

  if (bldc_step < 1 || bldc_step > 6) {
    step = 0;
  }
  else if ( bldc_direction == CW ) {
    step = cw[bldc_step - 1];
  }
  else if ( bldc_direction == CCW ) {
    step = ccw[bldc_step - 1];
  }
  else {
    step = 0;
  }

  switch(step) {
    case 1:
      // Step 1
      digitalWrite(PORT_in1, LOW);
      HwPWM0.writePin(PORT_en1, 0, false);
      digitalWrite(PORT_in2, HIGH);
      HwPWM0.writePin(PORT_en2, duty_cycle, false);
      digitalWrite(PORT_in3, LOW);
      HwPWM0.writePin(PORT_en3, duty_max, false);
      break;
    case 2:
      // Step 2
      digitalWrite(PORT_in1, HIGH);
      HwPWM0.writePin(PORT_en1, duty_cycle, false);
      digitalWrite(PORT_in2, LOW);
      HwPWM0.writePin(PORT_en2, 0, false);
      digitalWrite(PORT_in3, LOW);
      HwPWM0.writePin(PORT_en3, duty_max, false);
      break;
    case 3:
      // Step 3
      digitalWrite(PORT_in1, HIGH);
      HwPWM0.writePin(PORT_en1, duty_cycle, false);
      digitalWrite(PORT_in2, LOW);
      HwPWM0.writePin(PORT_en2, duty_max, false);
      digitalWrite(PORT_in3, LOW);
      HwPWM0.writePin(PORT_en3, 0, false);
      break;
    case 4:
      // Step 4
      digitalWrite(PORT_in1, LOW);
      HwPWM0.writePin(PORT_en1, 0, false);
      digitalWrite(PORT_in2, LOW);
      HwPWM0.writePin(PORT_en2, duty_max, false);
      digitalWrite(PORT_in3, HIGH);
      HwPWM0.writePin(PORT_en3, duty_cycle, false);
      break;
    case 5:
      // Step 5
      digitalWrite(PORT_in1, LOW);
      HwPWM0.writePin(PORT_en1, duty_max, false);
      digitalWrite(PORT_in2, LOW);
      HwPWM0.writePin(PORT_en2, 0, false);
      digitalWrite(PORT_in3, HIGH);
      HwPWM0.writePin(PORT_en3, duty_cycle, false);
      break;
    case 6:
      // Step 6
      digitalWrite(PORT_in1, LOW);
      HwPWM0.writePin(PORT_en1, duty_max, false);
      digitalWrite(PORT_in2, HIGH);
      HwPWM0.writePin(PORT_en2, duty_cycle, false);
      digitalWrite(PORT_in3, LOW);
      HwPWM0.writePin(PORT_en3, 0, false);
      break;
    default:
      bldc_idle();
  }
}

void bldc_idle() {
  digitalWrite(PORT_in1, LOW);
  HwPWM0.writePin(PORT_en1, 0, false);
  digitalWrite(PORT_in2, LOW);
  HwPWM0.writePin(PORT_en2, 0, false);
  digitalWrite(PORT_in3, LOW);
  HwPWM0.writePin(PORT_en3, 0, false);

  onePixel.setPixelColor(0, 255, 192, 203);
  onePixel.show();
}

void bldc_brake() {
  digitalWrite(PORT_in1, LOW);
  HwPWM0.writePin(PORT_en1, duty_max, false);
  digitalWrite(PORT_in2, LOW);
  HwPWM0.writePin(PORT_en2, duty_max, false);
  digitalWrite(PORT_in3, LOW);
  HwPWM0.writePin(PORT_en3, duty_max, false);

  onePixel.setPixelColor(0, 255, 191, 0);
  onePixel.show();
}

void bldc_regen() {
  digitalWrite(PORT_in1, LOW);
  HwPWM0.writePin(PORT_en1, duty_cycle_regen, false);
  digitalWrite(PORT_in2, LOW);
  HwPWM0.writePin(PORT_en2, duty_cycle_regen, false);
  digitalWrite(PORT_in3, LOW);
  HwPWM0.writePin(PORT_en3, duty_cycle_regen, false);

  onePixel.setPixelColor(0, 0, 255, 0);
  onePixel.show();
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
  onePixel.setPixelColor(0, 255, 192, 203);
  onePixel.show();
}

void turnOffHV() {
  hvgate_on = false;
  digitalWrite(PORT_hvgate, hvgate_on);
  onePixel.clear();
  onePixel.show();
}