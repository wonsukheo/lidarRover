#include <Arduino.h>
#include <Sabertooth.h>

// ================= HARDWARE & DRIVER SETUP =================
// Sabertooth Drivers on Hardware Serial1 (Due Pin 18 TX1)
Sabertooth FrontDriver(130, Serial1);
Sabertooth RearDriver(128, Serial1);

// Physical Dimensions & Specs
const float WHEEL_BASE     = 0.47;   // Track width between left and right wheels (meters)
const float WHEEL_RADIUS   = 0.127;  // Wheel radius (meters)
const int   TICKS_PER_REV  = 90;     // Encoder ticks per wheel revolution

// Encoder Filtering (Microseconds)
const uint32_t DEBOUNCE_MICROS = 150;

// PID Tuning Gains
float Kp = 2.5, Ki = 0.5, Kd = 0.05;

// Safety Watchdog
const unsigned long TIMEOUT_MS = 500;
unsigned long last_cmd_time = 0;

// Encoder Pin Definitions
const byte FL_PIN_A = 2;
const byte FL_PIN_B = 4;
const byte FR_PIN_A = 3;
const byte FR_PIN_B = 5;
const byte RL_PIN_A = 21;
const byte RL_PIN_B = 6;
const byte RR_PIN_A = 20;
const byte RR_PIN_B = 7;

// ================= VOLATILE ENCODER COUNTS =================
volatile int32_t FLcount = 0;
volatile int32_t FRcount = 0;
volatile int32_t RLcount = 0;
volatile int32_t RRcount = 0;

void isr_FL() {
  static uint32_t lastFL = 0;
  uint32_t now = micros();
  if (now - lastFL >= DEBOUNCE_MICROS) {
    if (digitalRead(FL_PIN_B) == LOW) FLcount++;
    else FLcount--;

    lastFL = now;
  }
}

void isr_RL() {
  static uint32_t lastRL = 0;
  uint32_t now = micros();
  if (now - lastRL >= DEBOUNCE_MICROS) {
    if (digitalRead(RL_PIN_B) == LOW) RLcount++;
    else RLcount--;

    lastRL = now;
  }
}

void isr_FR() {
  static uint32_t lastFR = 0;
  uint32_t now = micros();
  if (now - lastFR >= DEBOUNCE_MICROS) {
    if (digitalRead(FR_PIN_B) == LOW) FRcount--;
    else FRcount++;

    lastFR = now;
  }
}

void isr_RR() {
  static uint32_t lastRR = 0;
  uint32_t now = micros();
  if (now - lastRR >= DEBOUNCE_MICROS) {
    if (digitalRead(RR_PIN_B) == LOW) RRcount--;
    else RRcount++; 

    lastRR = now;
  }
}

// ================= MOTOR CONTROL ===========================
void setAllMotors(int cmd_left, int cmd_right) {
  cmd_left  = constrain(cmd_left, -127, 127);
  cmd_right = constrain(cmd_right, -127, 127);

  FrontDriver.motor(1, cmd_left);
  FrontDriver.motor(2, cmd_right);
  RearDriver.motor(1, cmd_left);
  RearDriver.motor(2, cmd_right);
}

void stopAllMotors() {
  FrontDriver.motor(1, 0);
  FrontDriver.motor(2, 0);
  RearDriver.motor(1, 0);
  RearDriver.motor(2, 0);
}

// ================= STATE & HELPERS =========================
float target_v_left = 0.0, target_v_right = 0.0;
float int_err_L = 0.0, prev_err_L = 0.0;
float int_err_R = 0.0, prev_err_R = 0.0;

unsigned long prev_pid_time = 0;
unsigned long prev_telemetry_time = 0;
long prev_left_ticks = 0, prev_right_ticks = 0;

void getAveragedTicks(long &out_left, long &out_right) {
  noInterrupts();
  long fl = FLcount, rl = RLcount;
  long fr = FRcount, rr = RRcount;
  interrupts();
  out_left  = (fl + rl) / 2;
  out_right = (fr + rr) / 2;
}

// SAM3X Hardware Glitch Filter for Due Pins
void enableDuePinFilter(byte pin) {
  g_APinDescription[pin].pPort->PIO_IFER = g_APinDescription[pin].ulPin;
  g_APinDescription[pin].pPort->PIO_DIFSR = g_APinDescription[pin].ulPin;
}

// ================= SETUP & LOOP ===========================
void setup() {
  Serial.begin(115200); // UART0 -> Raspberry Pi
  Serial1.begin(9600);   // UART1 -> Sabertooth Drivers

  delay(2000);          // Allow Sabertooth boards to finish booting
  FrontDriver.autobaud();
  RearDriver.autobaud();
  stopAllMotors();

  // Configure encoder pins
  pinMode(FL_PIN_A, INPUT_PULLUP);
  pinMode(FL_PIN_B, INPUT_PULLUP);
  pinMode(FR_PIN_A, INPUT_PULLUP);
  pinMode(FR_PIN_B, INPUT_PULLUP);
  pinMode(RL_PIN_A, INPUT_PULLUP);
  pinMode(RL_PIN_B, INPUT_PULLUP);
  pinMode(RR_PIN_A, INPUT_PULLUP);
  pinMode(RR_PIN_B, INPUT_PULLUP);

  // Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(FL_PIN_A), isr_FL, FALLING);
  attachInterrupt(digitalPinToInterrupt(FR_PIN_A), isr_FR, FALLING);
  attachInterrupt(digitalPinToInterrupt(RL_PIN_A), isr_RL, FALLING);
  attachInterrupt(digitalPinToInterrupt(RR_PIN_A), isr_RR, FALLING);

  // Enable SAM3X Hardware Glitch Filters
  enableDuePinFilter(FL_PIN_A);
  enableDuePinFilter(FL_PIN_B);
  enableDuePinFilter(FR_PIN_A);
  enableDuePinFilter(FR_PIN_B);
  enableDuePinFilter(RL_PIN_A);
  enableDuePinFilter(RL_PIN_B);
  enableDuePinFilter(RR_PIN_A);
  enableDuePinFilter(RR_PIN_B);
}

void loop() {
  unsigned long now = millis();

  // 1. Receive cmd_vel from Raspberry Pi
  if (Serial.available() > 0) {
    char header = Serial.read();
    if (header == 'v') {
      float linear_x  = Serial.parseFloat();
      float angular_z = Serial.parseFloat();

      // Differential Drive Kinematics
      target_v_left  = linear_x - (angular_z * WHEEL_BASE / 2.0);
      target_v_right = linear_x + (angular_z * WHEEL_BASE / 2.0);
      last_cmd_time = now;
    }
  }

  // 2. Safety Watchdog Timeout
  if (now - last_cmd_time > TIMEOUT_MS) {
    target_v_left  = 0.0;
    target_v_right = 0.0;
  }

  // 3. Velocity PID Loop (50 Hz / 20 ms)
  if (now - prev_pid_time >= 20) {
    float dt = (now - prev_pid_time) / 1000.0;
    prev_pid_time = now;

    long cur_L, cur_R;
    getAveragedTicks(cur_L, cur_R);

    float d_ticks_L = cur_L - prev_left_ticks;
    float d_ticks_R = cur_R - prev_right_ticks;
    prev_left_ticks  = cur_L;
    prev_right_ticks = cur_R;

    float actual_v_left  = ((d_ticks_L / (float)TICKS_PER_REV) * 2.0 * PI * WHEEL_RADIUS) / dt;
    float actual_v_right = ((d_ticks_R / (float)TICKS_PER_REV) * 2.0 * PI * WHEEL_RADIUS) / dt;

    // Left PID
    float err_L = target_v_left - actual_v_left;
    int_err_L += err_L * dt;
    float der_L = (err_L - prev_err_L) / dt;
    prev_err_L = err_L;
    float out_L = (Kp * err_L) + (Ki * int_err_L) + (Kd * der_L);

    // Right PID
    float err_R = target_v_right - actual_v_right;
    int_err_R += err_R * dt;
    float der_R = (err_R - prev_err_R) / dt;
    prev_err_R = err_R;
    float out_R = (Kp * err_R) + (Ki * int_err_R) + (Kd * der_R);

    // Anti-windup and stop override
    if (target_v_left == 0.0 && target_v_right == 0.0) {
      int_err_L = 0; int_err_R = 0;
      setAllMotors(0, 0);
    } else {
      setAllMotors((int)out_L, (int)out_R);
    }
  }

  // 4. Send Odometry Ticks to Raspberry Pi (20 Hz / 50 ms)
  if (now - prev_telemetry_time >= 50) {
    prev_telemetry_time = now;
    long send_L, send_R;
    getAveragedTicks(send_L, send_R);
    Serial.print("e ");
    Serial.print(send_L);
    Serial.print(" ");
    Serial.println(send_R);
  }
}
