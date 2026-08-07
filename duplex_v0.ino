#include <Servo.h>
// Encoder Pin config, reads value from wiring
const int leftWheelA = 2;  // GREEN (Hardware Interrupt)
const int leftWheelB = 4;  // PURPLE

const int rightWheelA = 3; // BLUE (Hardware Interrupt)
const int rightWheelB = 5; // PURPLE

// Encoder pulse counter, volatile ISR
volatile int32_t leftCount = 0;
volatile int32_t rightCount = 0;

// Servo setup
Servo FrontLeft; //S1 Motor Front?
Servo FrontRight; //S2?
Servo RearLeft; //S1?
Servo RearRight; //S2?

void setup(){
  // Attach Servos
  FrontLeft.attach(9);
  FrontRight.attach(10);
  RearLeft.attach(11);
  RearRight.attach(12);

  setmotors(90, 90);    // Rest Caliberation 
  
  // Enable internal pull-up resistors for active-low signals
  pinMode(leftWheelA, INPUT_PULLUP);
  pinMode(leftWheelB, INPUT_PULLUP);
  pinMode(rightWheelA, INPUT_PULLUP);
  pinMode(rightWheelB, INPUT_PULLUP);

  // Change interrupt trigger from RISING to FALLING
  attachInterrupt(digitalPinToInterrupt(leftWheelA), readLeftEncoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(rightWheelA), readRightEncoder, FALLING);

  Serial.begin(115200); // begin serial transfer
}

void loop() {
  // --- 1. Read Serial Data from Raspberry Pi ---
  // Expect 3 bytes packet: 1B start(0xFF) + 2B data(0-180)
  if (Serial.available() >= 3) {  
    if (Serial.peek() == 0xFF) {             // check header 0xFF
      Serial.read();                         // sink header
      uint8_t leftMotorVal = Serial.read();  // First data byte (L)
      uint8_t rightMotorVal = Serial.read();

      setmotors(leftMotorVal, rightMotorVal);
    } else { 
      Serial.read();    // sink invalid packet
    }
  }

  // --- 2. Send Encoder Feedback Back to Pi / Serial ---
  // Non-blocking timer: sends encoder counts every 100ms
  static uint32_t lastReport = 0;
  // 115,200 baud rate send ~15byte in 1.5ms 
  // we send 3*byte every 40ms, 25Hz
  if (millis() - lastReport >= 40) {
    lastReport = millis();

    // pause interrupt for reading
    noInterrupts();
    int32_t leftOut = leftCount;
    int32_t rightOut = rightCount;
    interrupts();

    // 1byte header + 4byte left + 4byte right
    Serial.write(0xFF);
    Serial.write((uint8_t*)&leftOut, sizeof(leftOut));
    Serial.write((uint8_t*)&rightOut, sizeof(rightOut));
  }
}

void readLeftEncoder() {
  if (digitalRead(leftWheelB) == LOW) leftCount++; 
  else leftCount--; 
}

void readRightEncoder() {
  if (digitalRead(rightWheelB) == LOW) rightCount--; 
  else rightCount++; 
}

void setmotors(int LeftSide, int RightSide){
  FrontLeft.write(LeftSide);
  FrontRight.write(RightSide);
  RearLeft.write(LeftSide);
  RearRight.write(RightSide);
}