#include <Motoron.h>
 
MotoronI2C mc(0x14); // Motor controller address

// Check iteration time
unsigned long ulStartTime;  
unsigned long ulEndTime;    
unsigned long ulDuration; 

// Debug
int iCounter = 0;
int iPwm;
float fOutput;

// LED
#define LED 13
volatile int vLed13 = 0;

// Pendulum angle encoder
#define ENCODER_A 2 // ChA // needs to be interrupt pin (2 or 3) // orange square cable
#define ENCODER_B 8 // ChB // yellow square cable
#define iCPR 2000 // Counts Per Revolution
volatile int vEncoderValue = 0;
float fAngle = 0;
float fAngleController = 0;

// PID gains
float fKp = 2.7;
float fKi = 0.2;
float fKd = 0.2;

//20.0 6.0 1.0 //sim better

//2.7 0.2 0.2 //PID good


// float fKp = 20.0; //simulation, bad
// float fKi = 0.05;
// float fKd = 1.0;

//20 0.05 1
 
float fPrevError = 0;
float fIntegral = 0;
float fDt = 0.008;

 
// Motor limits
float fVmax = 12;
float fRWheel = 0.065;
float fResistance = 5;
float fKt = 0.0892;
 
// PID control function
int iPidControl(float fTarget, float fCurrent) {
  float fError = fTarget - fCurrent;
  fIntegral += fError * fDt;
  float fDerivative = (fError - fPrevError) / fDt;
  fOutput = fKp * fError + fKi * fIntegral + fKd * fDerivative;
  fPrevError = fError;
 
  float fTau = fRWheel * fOutput;
  float fI = fTau / fKt;
  float fV = fResistance * fI;
  float fD = fV / fVmax;
  int iPwm = (int)(fD * 800) * -1 ;
  iPwm = constrain(iPwm, -800, 800);
  return iPwm;
}
 
void Controller() {
  fAngle = (float(vEncoderValue) / iCPR) * 360.0;
  if (fAngle > 180) fAngle -= 360;
  fAngleController = fAngle;
 
  int iPwm = iPidControl(0, fAngleController);
  mc.setSpeed(1, iPwm);
  mc.setSpeed(2, iPwm);
}

void display() { // Display information of the robot
  Serial.print("|A:");
  Serial.print(fAngleController);
 
  Serial.print("|u:");
  Serial.print(fOutput);
  Serial.print("|p:");
  Serial.println(iPwm);
}

void pendulum_encoder_voltage() {
  //read encoder pins A and B
  int iA = digitalRead(ENCODER_A);
  int iB = digitalRead(ENCODER_B);
 
  // If the state of A changed, it means the encoder has been rotated
  if ((iA == HIGH) != (iB == LOW)) {
    vEncoderValue--;
  } else {
    vEncoderValue++;
  }
}

void PendulumAngleMeasure() {
  // Calculate degrees of rotation
  fAngle = (float(vEncoderValue) / iCPR) * 360.0;
  fAngleController = fAngle;
 
  // Handle wrap-around for angle (0 to 360 degrees)
  if (fAngle < 0) {
    fAngle += 360;
  } else if (fAngle >= 360) {
    fAngle -= 360;
  }
 
  if (fAngleController > 180) {
    fAngleController -= 360;
  }
}

// No need for wheel encoder because PID does not look at that
 
void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to open

  pinMode(LED, OUTPUT);

  // Pendulum encoder
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), pendulum_encoder_voltage, CHANGE);
  
  // I2C for motor shield
  Wire.begin();
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
}
 
void loop() {
  ulStartTime = millis();

  PendulumAngleMeasure();
  Controller();
  ulEndTime = millis();
  ulDuration = ulEndTime - ulStartTime;

  Serial.print("t:");
  Serial.println(ulDuration);
 
  display();



  delay(8); // Might not be needed/changed
}





































































// #include <Motoron.h>
 
// MotoronI2C mc(0x14); // Motor controller address

// //check iteration time
// unsigned long ulStartTime;  
// unsigned long ulEndTime;    
// unsigned long ulDuration; 

// //debug
// int iCounter = 0;
// int iPwm;

// //LED
// #define LED 13
// volatile int vLed13 = 0;

// //pendulum angle encoder
// #define ENCODER_A 2 // ChA //needs to be interrupt pin (2 or 3) //orange square cable
// #define ENCODER_B 8 // ChB //yellow square cable
// #define iCPR 2000 // Counts Per Revolution
// volatile int vEncoderValue = 0;
// float fAngle = 0;
// float fAngleController = 0;

// //wheels encoder not used by PID

// //pendulum encoder func
// void pendulum_encoder_voltage() {
//   //read encoder pins A and B
//   int iA = digitalRead(ENCODER_A);
//   int iB = digitalRead(ENCODER_B);
 
//   // If the state of A changed, it means the encoder has been rotated
//   if ((iA == HIGH) != (iB == LOW)) {
//     vEncoderValue--;
//   } else {
//     vEncoderValue++;
//   }
// }


// //pid gains
// float Kp = 10.0;
// float Ki = 0.5;
// float Kd = 1.0;
 
// float prev_error = 0;
// float integral = 0;
// float dt = 0.008;
 
// // Motor limits
// float Vmax = 12;
// float r_wheel = 0.065;
// float resistance = 5;
// float Kt = 0.0892;
 
// // PID control function
// int pid_control(float target, float current) {
//   float error = target - current;
//   integral += error * dt;
//   float derivative = (error - prev_error) / dt;
//   float output = Kp * error + Ki * integral + Kd * derivative;
//   prev_error = error;
 
//   float tau = r_wheel * output;
//   float I = tau / Kt;
//   float V = resistance * I;
//   float D = V / Vmax;
//   int pwm = (int)(D * 800);
//   pwm = constrain(pwm, -800, 800);
//   return pwm;
// }
 
// void Controller() {
//   angle = (float(encoder_value) / CPR) * 360.0;
//   if (angle > 180) angle -= 360;
//   angle_controller = angle;
 
//   int pwm = pid_control(0, angle_controller);
//   mc.setSpeed(1, pwm);
//   mc.setSpeed(2, pwm);
// }

// void display() { //display information of the robot
//   Serial.print("W:");
//   Serial.print(fPos / 1000);
 
//   Serial.print("|A:");
//   Serial.print(fAngleController);
 
//   Serial.print("|u:");
//   Serial.print(fU);
//   Serial.print("|p:");
//   Serial.println(iPwm);
// }

// void PendulumAngleMeasure() {
//   // Calculate degrees of rotation
//     fAngle = (float(vEncoderValue) / iCPR) * 360.0;
//     fAngleController = fAngle;
 
//     // Handle wrap-around for angle (0 to 360 degrees)
//     if (fAngle < 0) {
//       fAngle += 360;
//     } else if (fAngle >= 360) {
//       fAngle -= 360;
//     }
 
//     if (fAngleController > 180){
//       fAngleController -= 360;
//     }
// }

// //no need for wheel encoder bc PID does not look at that
 
// void setup() {
//   Serial.begin(115200);
//   while (!Serial); //wait for serial port to open

//   pinMode(LED, OUTPUT);

//   //pendulum encoder
//   pinMode(ENCODER_A, INPUT_PULLUP);
//   pinMode(ENCODER_B, INPUT_PULLUP);
//   attachInterrupt(digitalPinToInterrupt(ENCODER_A), pendulum_encoder_voltage, CHANGE);
  
//   //I2C for motor shield
//   Wire.begin();
//   mc.reinitialize();
//   mc.disableCrc();
//   mc.clearResetFlag();
// }
 
// void loop() {
//   ulStartTime = millis();

//   LEDblink();
//   PendulumAngleMeasure();
//   Controller();
//   ulEndTime = millis();
//   ulDuration = ulEndTime - ulStartTime;
 
//   display();

//   delay(8); //might not be needed/changed

// }
 
 