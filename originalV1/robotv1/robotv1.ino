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

AccelStepper stepperBase(1, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperArm1(1, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepperArm2(1, Z_STEP_PIN, Z_DIR_PIN);
Servo gripper;

// Serial parsing buffer allocations
const unsigned int MAX_MESSAGE_LENGTH = 64;
static char message[MAX_MESSAGE_LENGTH];
static unsigned int message_pos = 0;

void setup() {
  Serial.begin(115200);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Force enable active-low stepper drivers

  // Performance parameters optimized for jointed link arms
  stepperBase.setMaxSpeed(1200);   stepperBase.setAcceleration(600);
  stepperArm1.setMaxSpeed(1200);   stepperArm1.setAcceleration(600);
  stepperArm2.setMaxSpeed(1200);   stepperArm2.setAcceleration(600);
  
  gripper.attach(GRIPPER_PIN);
  gripper.write(0); // Initialize open position
}

void processCommand(char* cmd) {
  // Commands formatted as: "M base_steps arm1_steps arm2_steps gripper_state"
  if (cmd[0] == 'M') {
    long b, a1, a2;
    int g;
    
    // Parse safely out of raw character array
    if (sscanf(cmd, "M %ld %ld %ld %d", &b, &a1, &a2, &g) == 4) {
      
      // Assign physical targets
      stepperBase.moveTo(b);
      stepperArm1.moveTo(a1);
      stepperArm2.moveTo(a2);

      // Explicit blocking loop until targets are successfully met
      while (stepperBase.distanceToGo() != 0 || 
             stepperArm1.distanceToGo() != 0 || 
             stepperArm2.distanceToGo() != 0) {
        stepperBase.run();
        stepperArm1.run();
        stepperArm2.run();
      }

      // 1 = Grab (90 deg), 0 = Release (0 deg)
      gripper.write(g == 1 ? 90 : 0); 
      
      // Increased to 500ms to allow physical micro-servo teeth to fully clamp/retract
      delay(500); 
      
      // Transmit explicit completion handshake back to Python loop
      Serial.println("OK");
    }
  }
}

void loop() {
  // Non-blocking serial stream processing
  while (Serial.available() > 0) {
    char inByte = Serial.read();
    
    if (inByte != '\n') {
      if (message_pos < MAX_MESSAGE_LENGTH - 1) {
        message[message_pos] = inByte;
        message_pos++;
      }
    } else {
      // Terminate character string array and pass to processor
      message[message_pos] = '\0';
      processCommand(message);
      
      // Reset index buffer tracking
      message_pos = 0;
    }
  }
  
  // Maintain active motor positioning/holding torque when idle
  stepperBase.run();
  stepperArm1.run();
  stepperArm2.run();
}

