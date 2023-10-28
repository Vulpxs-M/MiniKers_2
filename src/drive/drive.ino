#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // for Serial

#define CW    1
#define CCW  -1

// Port Assignment
int PORT_in1 = 26;
int PORT_in2 = 24;
int PORT_in3 = 0;
int PORT_en1 = 25;
int PORT_en2 = 1;
int PORT_en3 = 22;
int PORT_nfault = 5;
int PORT_nreset = 23;

int PORT_hall_u = 11;
int PORT_hall_v = 12;
int PORT_hall_w = 13;

int PORT_vsense = A1;
int PORT_isense = A0;
int PORT_cur1 = A5;
int PORT_cur2 = 18;
int PORT_cur3 = 17;

int PORT_sw1 = 6;

// Values for Serial Print
int val_nfault = 0;

int val_u = 0;
int val_v = 0;
int val_w = 0;

int vin_raw = 0;
float mv_per_lsb = 3600.0F/1024.0F/0.129F; // 10-bit ADC with 3.6V input range + 0.129 V/unit
int iin_raw = 0;
float ma_per_lsb = 3600.0F/1024.0F/0.1765F;
int val_cur1 = 0;
int val_cur2 = 0;
int val_cur3 = 0;

// Configuration and Global Parameters
int duty_cycle = bit(7) - 1;
// bit(7) - 1 = 127 in 0-255 -> 50%; bit(6) - 1 = 63 -> 25%
int bldc_direction = CW;
int bldc_step = 0;

// For RPM and Pulse Count
int direct = CW;
int pulseCount;

float startTime;
float prevTime;
float pulseTimeW;
float pulseTimeU;
float pulseTimeV;
float avgPulseTime;

float PPM;
float RPM;

// Setup
void setup() {
  // Digital Pins
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

  // For nrf52840 with native usb
  Serial.begin(115200);
  while ( !Serial ) delay(10);

  // PWM Configuration
  HwPWM0.addPin(PORT_in1);
  HwPWM0.addPin(PORT_in2);
  HwPWM0.addPin(PORT_in3);

  HwPWM0.begin();
  HwPWM0.setResolution(8);
  HwPWM0.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_8);

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
}

void loop() {
  if ((millis() - prevTime) > 600) RPM = 0; 
  Serial.println(String("Hall Step: ") + bldc_step);
  Serial.println(String("RPM: ") + RPM);
  Serial.println(String("Pulse Count: ") + pulseCount);
  Serial.println(String("Detected Direction: ") + direct);
  Serial.println(String("Control Direction: ") + bldc_direction);
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

  // Delay for Detection and Sensing
  delay(100);
  */
}

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

void hall_drive_w() {
  startTime = millis();
  hall_read();
  direct = (val_w == val_v) ? CCW : CW;
  pulseTimeW = startTime - prevTime;
  calculate_rpm();
  bldc_move();
}

void hall_drive_v() {
  startTime = millis();
  hall_read();
  direct = (val_v == val_u) ? CCW : CW;
  pulseTimeV = startTime - prevTime;
  calculate_rpm();
  bldc_move();
}

void hall_drive_u() {
  startTime = millis();
  hall_read();
  direct = (val_u == val_w) ? CCW : CW;
  pulseTimeU = startTime - prevTime;
  calculate_rpm();
  bldc_move();
}

void bldc_move() {
  switch(convert(bldc_step, bldc_direction)) {
    case 1:
      // Step 1
      HwPWM0.writePin(PORT_in1, 0, false);
      digitalWrite(PORT_en1, LOW);
      HwPWM0.writePin(PORT_in2, duty_cycle, false);
      digitalWrite(PORT_en2, HIGH);
      HwPWM0.writePin(PORT_in3, 0, false);
      digitalWrite(PORT_en3, HIGH);
      break;
    case 2:
      // Step 2
      HwPWM0.writePin(PORT_in1, duty_cycle, false);
      digitalWrite(PORT_en1, HIGH);
      HwPWM0.writePin(PORT_in2, 0, false);
      digitalWrite(PORT_en2, LOW);
      HwPWM0.writePin(PORT_in3, 0, false);
      digitalWrite(PORT_en3, HIGH);
      break;
    case 3:
      // Step 3
      HwPWM0.writePin(PORT_in1, duty_cycle, false);
      digitalWrite(PORT_en1, HIGH);
      HwPWM0.writePin(PORT_in2, 0, false);
      digitalWrite(PORT_en2, HIGH);
      HwPWM0.writePin(PORT_in3, 0, false);
      digitalWrite(PORT_en3, LOW);
      break;
    case 4:
      // Step 4
      HwPWM0.writePin(PORT_in1, 0, false);
      digitalWrite(PORT_en1, LOW);
      HwPWM0.writePin(PORT_in2, 0, false);
      digitalWrite(PORT_en2, HIGH);
      HwPWM0.writePin(PORT_in3, duty_cycle, false);
      digitalWrite(PORT_en3, HIGH);
      break;
    case 5:
      // Step 5
      HwPWM0.writePin(PORT_in1, 0, false);
      digitalWrite(PORT_en1, HIGH);
      HwPWM0.writePin(PORT_in2, 0, false);
      digitalWrite(PORT_en2, LOW);
      HwPWM0.writePin(PORT_in3, duty_cycle, false);
      digitalWrite(PORT_en3, HIGH);
      break;
    case 6:
      // Step 6
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

// Converts from Hall Step to Commutation Step
int convert(int hall_step, int direction) {
  int cw[] = {2, 4, 3, 6, 1, 5};
  int ccw[] = {1, 3, 2, 5, 6, 4};

  if (hall_step < 1 || hall_step > 6)
    return 0;

  if (direction == CW) {
    return cw[hall_step - 1];
  } else if (direction == CCW) {
    return ccw[hall_step - 1];
  } else {
    return 0;
  }
}

void buttonPress() {
  if ( bldc_direction == CW ) {
    bldc_direction = CCW;
  }
  else if ( bldc_direction == CCW ) {
    bldc_direction = CW;
  }
}