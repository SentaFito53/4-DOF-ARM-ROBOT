#include <Braccio.h>
#include <Servo.h>

// Robot parameters (harus sama dengan GUI)
const float L1 = 125.0;    // Link 1 length (mm)
const float L2 = 125.0;    // Link 2 length (mm)  
const float L3 = 195.0;    // Link 3 length (mm)
const float BASE_HEIGHT = 0.0;  // Base height (mm)

// Joint angles in degrees
float theta1, theta2, theta3, theta4;

// Braccio servos
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_ver;
Servo wrist_rot;
Servo gripper;

// Serial communication
String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  inputString.reserve(200);
  
  // Initialize Braccio
  Braccio.begin();
  
  // Initial position
  goToHome();
  
  Serial.println("BRACCIO 4DOF ROBOT READY - FIXED ANGLE CONVENTION");
  Serial.println("Format: X200.0Y150.0Z100.0R0.0");
  Serial.println("Simulation 0° = Braccio 90° (Vertical)");
}

void loop() {
  if (stringComplete) {
    processCoordinates();
    inputString = "";
    stringComplete = false;
  }
  delay(10);
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void processCoordinates() {
  float x, y, z, roll;
  
  // Parse: X200.0Y150.0Z100.0R0.0
  int xPos = inputString.indexOf('X');
  int yPos = inputString.indexOf('Y');
  int zPos = inputString.indexOf('Z');
  int rPos = inputString.indexOf('R');
  
  if (xPos >= 0 && yPos > xPos && zPos > yPos && rPos > zPos) {
    x = inputString.substring(xPos+1, yPos).toFloat();
    y = inputString.substring(yPos+1, zPos).toFloat();
    z = inputString.substring(zPos+1, rPos).toFloat();
    roll = inputString.substring(rPos+1).toFloat();
    
    Serial.print("Target: X=");
    Serial.print(x);
    Serial.print(" Y=");
    Serial.print(y);
    Serial.print(" Z=");
    Serial.print(z);
    Serial.print(" R=");
    Serial.println(roll);
    
    if (inverseKinematics(x, y, z, roll)) {
      executeMovement();
    } else {
      Serial.println("UNREACHABLE POSITION");
    }
  }
}

bool inverseKinematics(float x, float y, float z, float roll_deg) {
  // Convert to cylindrical coordinates
  float r = sqrt(x*x + y*y);
  float h = z - BASE_HEIGHT;
  
  // Base angle
  theta1 = atan2(y, x) * 180.0 / PI;
  
  // Wrist position
  float roll_rad = roll_deg * PI / 180.0;
  float wrist_x = r - L3 * cos(roll_rad);
  float wrist_z = h - L3 * sin(roll_rad);
  
  float d = sqrt(wrist_x*wrist_x + wrist_z*wrist_z);
  
  // Reachability check
  if (d > (L1 + L2) || d < fabs(L1 - L2)) {
    return false;
  }
  
  // Elbow angle
  float cos_theta3 = (d*d - L1*L1 - L2*L2) / (2 * L1 * L2);
  cos_theta3 = constrain(cos_theta3, -1.0, 1.0);
  theta3 = acos(cos_theta3) * 180.0 / PI;
  
  // Shoulder angle
  float alpha = atan2(wrist_z, wrist_x) * 180.0 / PI;
  float beta = atan2(L2 * sin(theta3 * PI/180.0), 
                    L1 + L2 * cos(theta3 * PI/180.0)) * 180.0 / PI;
  theta2 = alpha - beta;
  
  // Wrist angle
  theta4 = roll_deg - (theta2 + theta3);
  
  Serial.print("IK Angles (Simulation Convention): ");
  Serial.print("Base=");
  Serial.print(theta1);
  Serial.print(" Shoulder=");
  Serial.print(theta2);
  Serial.print(" Elbow=");
  Serial.print(theta3);
  Serial.print(" Wrist=");
  Serial.println(theta4);
  
  return true;
}

void executeMovement() {
  // KONVERSI SUDUT: Simulasi → Braccio Servo
  // Simulasi: 0° = sejajar sumbu Y (vertikal)
  // Braccio: 90° = vertikal
  
  // Base (M1) - sama karena rotasi
  int base_angle = theta1 + 90;  // Konversi ke range 0-180
  base_angle = constrain(base_angle, 0, 180);
  
  // Shoulder (M2) - konversi: simulasi 0° = servo 90°
  int shoulder_angle = theta2;  // Invert direction
  shoulder_angle = constrain(shoulder_angle, 15, 165);
  
  // Elbow (M3) - konversi: simulasi 0° = servo 90°
  int elbow_angle = 90 + theta3;  // Same direction
  elbow_angle = constrain(elbow_angle, 0, 180);
  
  // Wrist vertical (M4) - konversi: simulasi 0° = servo 90°
  int wrist_angle = 90 + theta4;  // Invert direction
  wrist_angle = constrain(wrist_angle, 0, 180);
  
  // Wrist rotation (M5) - fixed untuk sekarang
  int wrist_rot_angle = 90;
  wrist_rot_angle = constrain(wrist_rot_angle, 0, 180);
  
  // Gripper (M6) - closed
  int gripper_angle = 73;
  gripper_angle = constrain(gripper_angle, 10, 73);
  
  Serial.print("Servo Angles (Braccio): ");
  Serial.print("Base=");
  Serial.print(base_angle);
  Serial.print(" Shoulder=");
  Serial.print(shoulder_angle);
  Serial.print(" Elbow=");
  Serial.print(elbow_angle);
  Serial.print(" Wrist=");
  Serial.print(wrist_angle);
  Serial.print(" WristRot=");
  Serial.print(wrist_rot_angle);
  Serial.print(" Gripper=");
  Serial.println(gripper_angle);
  
  // Smooth movement
  Braccio.ServoMovement(20, base_angle, shoulder_angle, elbow_angle, 
                       wrist_angle, wrist_rot_angle, gripper_angle);
}

void goToHome() {
  // Home position dalam konvensi simulasi: semua joint 0°
  // Dikonversi ke Braccio: semua servo 90° (kecuali gripper)
  Serial.println("Going to HOME position");
  Serial.println("Simulation: (0°, 0°, 0°, 0°)");
  Serial.println("Braccio: (90°, 90°, 90°, 90°)");
  Braccio.ServoMovement(20, 90, 90, 90, 90, 90, 73);
  delay(1000);
}