#include <Servo.h>
// Encoder Pin Config
const int leftWheelA = 2;  // GREEN (Hardware Interrupt)
const int leftWheelB = 4;  // PURPLE

const int rightWheelA = 3; // BLUE (Hardware Interrupt)
const int rightWheelB = 5; // PURPLE

// Encoder pulse counter, volatile ISR
volatile int32_t leftEncoderCount = 0;
volatile int32_t rightEncoderCount = 0;

// Servo setup
Servo FrontLeft, FrontRight, RearLeft, RearRight;

void setup(){
  // Servo connect
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
  // 1. Read Serial Data from processor, (0XFF + 2byte data(0-180) x2)
  if (Serial.available() >= 3) {  
    if (Serial.peek() == 0xFF) {              // check header 0xFF
      Serial.read();                          // sink header

      int leftMotor = Serial.read();  // First data byte (L)
      int rightMotor = Serial.read();

      setmotors(leftMotor, rightMotor);
    } else { 
      Serial.read();   		              // sink invalid packet
    }
  }

  // 2. Send Encoder Feedback Back to processor via Serial
  // 115,200 baud rate send ~15byte in 1.5ms, 20ms==50hz
  static uint32_t lastReport = 0;
  
  if (millis() - lastReport >= 20) {
    lastReport = millis();

    // pause interrupt for reading
    noInterrupts();
    int32_t leftEncoder = leftEncoderCount;
    int32_t rightEncoder = rightEncoderCount;
    interrupts();

    // 1byte header + 4byte left + 4byte right
    Serial.write(0xFF);
    Serial.write((const uint8_t*)&leftEncoder, sizeof(leftEncoder));
    Serial.write((const uint8_t*)&rightEncoder, sizeof(rightEncoder));
  }
}

void readLeftEncoder() {
  if (digitalRead(leftWheelB) == LOW) leftEncoderCount++; 
  else leftEncoderCount--; 
}

void readRightEncoder() {
  if (digitalRead(rightWheelB) == LOW) rightEncoderCount--; 
  else rightEncoderCount++; 
}

void setmotors(int leftMotor, int rightMotor){
  FrontLeft.write(leftMotor);
  FrontRight.write(rightMotor);
  RearLeft.write(leftMotor);
  RearRight.write(rightMotor);
}
