#include <AccelStepper.h>
#include <Servo.h>

// Driver Pins (RAMPS 1.4 / CNC Shield standard)
#define X_STEP_PIN 2
#define X_DIR_PIN  5
#define Y_STEP_PIN 3
#define Y_DIR_PIN  6
#define Z_STEP_PIN 4
#define Z_DIR_PIN  7
#define ENABLE_PIN 8
#define GRIPPER_PIN 11

// CNC Shield Limit Pins
#define X_LIMIT_PIN 9
#define Y_LIMIT_PIN 10
#define Z_LIMIT_PIN 12 // Pin 11 used by Servo

AccelStepper stepperBase(1, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperArm1(1, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepperArm2(1, Z_STEP_PIN, Z_DIR_PIN);
Servo gripper;

const unsigned int MAX_MESSAGE_LENGTH = 64;
static char message[MAX_MESSAGE_LENGTH];
static unsigned int message_pos = 0;

// Number of microsteps to back away from switches after homing to clear the click
const int BACKOFF_STEPS = 200; 

void setup() {
  Serial.begin(115200);
  
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Enable active-low stepper drivers

  pinMode(X_LIMIT_PIN, INPUT_PULLUP);
  pinMode(Y_LIMIT_PIN, INPUT_PULLUP);
  pinMode(Z_LIMIT_PIN, INPUT_PULLUP);

  // Safe homing speeds
  stepperBase.setMaxSpeed(400);     stepperBase.setAcceleration(200);
  stepperArm1.setMaxSpeed(400);     stepperArm1.setAcceleration(200);
  stepperArm2.setMaxSpeed(400);     stepperArm2.setAcceleration(200);

  // 1. Home Arm 2 (Z Axis / Upper link)
  while(digitalRead(Z_LIMIT_PIN) == HIGH) {
    stepperArm2.moveTo(-10000); 
    stepperArm2.run();
  }
  // Clear the Z limit switch click
  stepperArm2.setCurrentPosition(0);
  stepperArm2.runToNewPosition(BACKOFF_STEPS); 
  stepperArm2.setCurrentPosition(0); 

  // 2. Home Arm 1 (Y Axis / Lower link)
  while(digitalRead(Y_LIMIT_PIN) == HIGH) {
    stepperArm1.moveTo(10000);
    stepperArm1.run();
  }
  // Clear the Y limit switch click
  stepperArm1.setCurrentPosition(0);
  stepperArm1.runToNewPosition(-BACKOFF_STEPS); // Moves opposite to homing direction
  stepperArm1.setCurrentPosition(0); 

  // 3. Home the Base (X Axis / Rotation)
  while(digitalRead(X_LIMIT_PIN) == HIGH) {
    stepperBase.moveTo(10000);
    stepperBase.run();
  }
  // Clear the X limit switch click
  stepperBase.setCurrentPosition(0);
  stepperBase.runToNewPosition(-BACKOFF_STEPS); // Moves opposite to homing direction
  stepperBase.setCurrentPosition(0); 

  // Unlocked target gameplay speeds
  stepperBase.setMaxSpeed(1200);   stepperBase.setAcceleration(600);
  stepperArm1.setMaxSpeed(1200);   stepperArm1.setAcceleration(600);
  stepperArm2.setMaxSpeed(1200);   stepperArm2.setAcceleration(600);
  
  gripper.attach(GRIPPER_PIN);
  gripper.write(0); 
}

void processCommand(char* cmd) {
  if (cmd[0] == 'M') {
    long b, a1, a2;
    int g;
    
    if (sscanf(cmd, "M %ld %ld %ld %d", &b, &a1, &a2, &g) == 4) {
      stepperBase.moveTo(b);
      stepperArm1.moveTo(a1);
      stepperArm2.moveTo(a2);

      while (stepperBase.distanceToGo() != 0 || 
             stepperArm1.distanceToGo() != 0 || 
             stepperArm2.distanceToGo() != 0) {
        stepperBase.run();
        stepperArm1.run();
        stepperArm2.run();
      }

      gripper.write(g == 1 ? 90 : 0); 
      delay(500); 
      Serial.println("OK"); 
    }
  }
}

void loop() {
  while (Serial.available() > 0) {
    char inByte = Serial.read();
    
    if (inByte != '\n') {
      if (message_pos < MAX_MESSAGE_LENGTH - 1) {
        message[message_pos] = inByte;
        message_pos++;
      }
    } else {
      message[message_pos] = '\0';
      processCommand(message);
      message_pos = 0;
    }
  }
  
  stepperBase.run();
  stepperArm1.run();
  stepperArm2.run();
}
