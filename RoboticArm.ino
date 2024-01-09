#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// Create an instance of the Adafruit_PWMServoDriver class
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo parameters
int baseServo = 0;
int shoulderServo_left = 1;
int shoulderServo_right = 2;
int elbowServo = 3;
int wristServo = 4;
int gripServo = 5;

// MG90s pulse width range (130-490) (150-650) (102-491)
int pwmMin = 130;
int pwmMax = 490;

// Arm parameters
double armX = 200;
double armY = 0;
double shoulderPart = 83.2;
double elbowPart = 128;
double shoulderJointHeight = 80;


void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
}

void loop() {
  homePosition();
  
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    int commaIndex = data.indexOf(',');
    
    // If the serial information includes a comma and is constant for 4 seconds
    if (commaIndex != -1 && checkStability() == true) {
      int cX = data.substring(0, commaIndex).toInt();
      int cY = data.substring(commaIndex + 1).toInt() + 59;
      int cZ = 16;

      Serial.println(data);

      objectPosition(cX, cY, cZ);
      delay(4000);
      dropPosition();
      delay(3000);
      homePosition();
    }
  }
}


void setServoPosition(int servoNum, int angle) {
  // Map the given angle to the servo's PWM range
  int pwmValue = map(angle, 0, 180, pwmMin, pwmMax);

  // Set the final servo position
  pwm.setPWM(servoNum, 0, pwmValue);
}

void openGrip(){
  setServoPosition(gripServo, 0);
}

void closeGrip(){
  setServoPosition(gripServo, 180);
}

void homePosition(){
  setServoPosition(baseServo, 110);
  setServoPosition(shoulderServo_left, 180);
  setServoPosition(shoulderServo_right, 0);
  setServoPosition(elbowServo, 50);
  setServoPosition(wristServo, 170);
}

void dropPosition(){
  setServoPosition(baseServo, 180);
  setServoPosition(shoulderServo_left, 150);
  setServoPosition(shoulderServo_right, 30);
  setServoPosition(elbowServo, 80);
  setServoPosition(wristServo, 170);

  delay(2000);
  openGrip();
}

// Checks if the centre point stays constant for 4 seconds (i.e. the object is not moving)
bool checkStability(){
  unsigned long startTime = millis();
  int initialCX = 0;
  int initialCY = 0;
  
  while (millis() - startTime < 2000) {
    if (Serial.available()) {
      String data = Serial.readStringUntil('\n');
      int commaIndex = data.indexOf(',');

      if (commaIndex != -1) {
        int cX = data.substring(0, commaIndex).toInt();
        int cY = data.substring(commaIndex + 1).toInt();
        
        if (initialCX == 0 && initialCY == 0) {
          initialCX = cX;
          initialCY = cY;
        } 
        else if (cX != initialCX || cY != initialCY) {
          // Values changed
          return false;
        }
      }
    }
    
    else {
      // Lost serial communication
      return false;
    }
  }
  
  // Values did not change
  return true;
}

double radiansToDegrees(float radians) {
  return radians * 180.0 / PI;
}

// Inverse Kinematics for robotic arm's servo motors to rotate to the object's position
void objectPosition(int cX, int cY, int cZ){
  // Base angle calculations
  double L = sqrt(pow(cY, 2) + pow(armX - cX, 2));
  double baseAngle = acos(constrain(cY / L, -1.0, 1.0));
  baseAngle = radiansToDegrees(baseAngle);
  setServoPosition(baseServo, 110 + baseAngle);

  // shoulder and elbow angle calculations 
  double D = sqrt(pow(L, 2) + pow(shoulderJointHeight - cZ, 2));
  double shoulderAngle = acos((pow(D, 2) + pow(shoulderPart, 2) - pow(elbowPart, 2))/(2*D*shoulderPart));
  shoulderAngle = radiansToDegrees(shoulderAngle);
  double elbowAngle = acos((pow(shoulderPart, 2) + pow(elbowPart, 2) - pow(D, 2))/(2*shoulderPart*elbowPart));
  elbowAngle = radiansToDegrees(elbowAngle);
  setServoPosition(shoulderServo_left, 20 + shoulderAngle);
  setServoPosition(shoulderServo_right, 180 - (20 + shoulderAngle));
  setServoPosition(elbowServo, 160 + elbowAngle);
  setServoPosition(wristServo, 170);

  delay(2000);
  closeGrip();
}

