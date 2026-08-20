#include <Servo.h>

// Servo Setup
Servo FrontLeft;   // Pin 9
Servo FrontRight;  // Pin 10
Servo RearLeft;    // Pin 11
Servo RearRight;   // Pin 12

// Servo Speed Definitions (Standard continuous rotation servos: 90 = STOP)
const int STOP_SPEED  = 90;
const int SLOW_SPEED  = 100; // Adjust between 91-120 depending on your motors' deadband

// Encoder Pin Configuration
const int flWheelA = 2;  // Front-Left   Channel A
const int flWheelB = 4;  // Front-Left   Channel B

const int frWheelA = 3;  // Front-Right  Channel A
const int frWheelB = 5;  // Front-Right  Channel B

const int rlWheelA = 6;  // Rear-Left    Channel A (Port D / PCINT22)
const int rlWheelB = 7;  // Rear-Left    Channel B

const int rrWheelA = 8;  // Rear-Right   Channel A (Port B / PCINT0)
const int rrWheelB = 13; // Rear-Right   Channel B

// Volatile encoder counters for ISR handling
volatile int32_t flEncoderCount = 0;
volatile int32_t frEncoderCount = 0;
volatile int32_t rlEncoderCount = 0;
volatile int32_t rrEncoderCount = 0;

// State Machine Variables for Non-blocking 3-second Timer
bool isRunning = false;
uint32_t runStartTime = 0;

void setup() {
  Serial.begin(115200);

  // Attach servos
  FrontLeft.attach(9);
  FrontRight.attach(10);
  RearLeft.attach(11);
  RearRight.attach(12);

  // Initialize motors in STOP state
  setmotors(STOP_SPEED, STOP_SPEED);

  // Enable internal pull-up resistors for encoder pins
  pinMode(flWheelA, INPUT_PULLUP);
  pinMode(flWheelB, INPUT_PULLUP);
  pinMode(frWheelA, INPUT_PULLUP);
  pinMode(frWheelB, INPUT_PULLUP);

  pinMode(rlWheelA, INPUT_PULLUP);
  pinMode(rlWheelB, INPUT_PULLUP);
  pinMode(rrWheelA, INPUT_PULLUP);
  pinMode(rrWheelB, INPUT_PULLUP);

  // Hardware Interrupts for Front Wheels (Pins 2 & 3)
  attachInterrupt(digitalPinToInterrupt(flWheelA), readFL_Encoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(frWheelA), readFR_Encoder, FALLING);

  // Pin Change Interrupts for Rear Wheels (Pins 6 & 8)
  PCICR  |= (1 << PCIE2);    // Port D vector
  PCICR  |= (1 << PCIE0);    // Port B vector

  PCMSK2 |= (1 << PCINT22); // Pin 6 (RL Channel A)
  PCMSK0 |= (1 << PCINT0);  // Pin 8 (RR Channel A)

  Serial.println(F("=============================================="));
  Serial.println(F("3-Second Motor Timed Test"));
  Serial.println(F("Type 's' -> Reset counts & drive straight for 3s"));
  Serial.println(F("Type 'p' -> Emergency STOP"));
  Serial.println(F("Type 'r' -> RESET encoder counts to 0 manually"));
  Serial.println(F("=============================================="));
  Serial.println(F("FL\tFR\tRL\tRR"));
}

void loop() {
  // 1. Process Serial Commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 's' || cmd == 'S') {
      // Reset encoder counts before starting
      resetEncoderCounts();

      // Start 3-second drive sequence
      isRunning = true;
      runStartTime = millis();
      setmotors(SLOW_SPEED, SLOW_SPEED);
      Serial.println(F("\n--- Driving Straight for 3 Seconds ---"));
    } 
    else if (cmd == 'p' || cmd == 'P') {
      // Immediate manual override stop
      isRunning = false;
      setmotors(STOP_SPEED, STOP_SPEED);
      Serial.println(F("\n--- Manual STOP ---"));
    } 
    else if (cmd == 'r' || cmd == 'R') {
      resetEncoderCounts();
      Serial.println(F("\n--- Counters RESET ---"));
    }
  }

  // 2. Timer Check: Stop motors automatically after 3000ms (3 seconds)
  if (isRunning && (millis() - runStartTime >= 3000)) {
    setmotors(STOP_SPEED, STOP_SPEED);
    isRunning = false;
    Serial.println(F("\n--- 3 Seconds Complete: Motors STOPPED ---"));
  }

  // 3. Print Live Encoder Counts (10 Hz)
  static uint32_t lastReport = 0;
  if (millis() - lastReport >= 100) {
    lastReport = millis();

    noInterrupts();
    int32_t fl = flEncoderCount;
    int32_t fr = frEncoderCount;
    int32_t rl = rlEncoderCount;
    int32_t rr = rrEncoderCount;
    interrupts();

    Serial.print(fl); Serial.print(F("\t"));
    Serial.print(fr); Serial.print(F("\t"));
    Serial.print(rl); Serial.print(F("\t"));
    Serial.println(rr);
  }
}
// Calibration Multipliers (Adjust these values until all 4 counts match)
float flTrim = 1.00; // Front Left
float frTrim = 1.07; // Front Right
float rlTrim = 1.70; // Rear Left (Increased to boost speed)
float rrTrim = 1.75; // Rear Right (Increased to boost speed)

// Motor Helper Function
void setmotors(int leftSpeed, int rightSpeed) {
  // Calculate speed offsets relative to 90 (Stop)
  int leftOffset  = leftSpeed - 90;
  int rightOffset = rightSpeed - 90;

  // Apply multipliers to the offsets
  int flVal = 90 + (leftOffset  * flTrim);
  int frVal = 90 + (rightOffset * frTrim);
  int rlVal = 90 + (leftOffset  * rlTrim);
  int rrVal = 90 + (rightOffset * rrTrim);

  // Constrain to valid servo range (0 to 180)
  FrontLeft.write(constrain(flVal, 0, 180));
  FrontRight.write(constrain(frVal, 0, 180));
  RearLeft.write(constrain(rlVal, 0, 180));
  RearRight.write(constrain(rrVal, 0, 180));
}

// Encoder Reset Helper Function
void resetEncoderCounts() {
  noInterrupts();
  flEncoderCount = 0;
  frEncoderCount = 0;
  rlEncoderCount = 0;
  rrEncoderCount = 0;
  interrupts();
}

// ----------------------------------------------------------------
// Interrupt Service Routines (ISRs)
// ----------------------------------------------------------------
void readFL_Encoder() {
  if (digitalRead(flWheelB) == LOW) flEncoderCount++;
  else flEncoderCount--;
}

void readFR_Encoder() {
  if (digitalRead(frWheelB) == LOW) frEncoderCount--;
  else frEncoderCount++;
}

ISR(PCINT2_vect) {
  static uint8_t lastPortD = PIND;
  uint8_t currentPortD = PIND;
  uint8_t changedBits = currentPortD ^ lastPortD;

  if ((changedBits & (1 << PD6)) && !(currentPortD & (1 << PD6))) {
    if (digitalRead(rlWheelB) == LOW) rlEncoderCount++;
    else rlEncoderCount--;
  }
  lastPortD = currentPortD;
}

ISR(PCINT0_vect) {
  static uint8_t lastPortB = PINB;
  uint8_t currentPortB = PINB;
  uint8_t changedBits = currentPortB ^ lastPortB;

  if ((changedBits & (1 << PB0)) && !(currentPortB & (1 << PB0))) {
    if (digitalRead(rrWheelB) == LOW) rrEncoderCount--;
    else rrEncoderCount++;
  }
  lastPortB = currentPortB;
}