#include <Motoron.h>
MotoronI2C mc(0x14); //address changed
 

//check iteration time
unsigned long ulStartTime;  
unsigned long ulEndTime;    
unsigned long ulDuration; 

//debug
int iCounter = 0;
int iPwm;

//LED
#define LED 13
volatile int vLed13 = 0;

//pendulum angle encoder
#define ENCODER_A 2 // ChA //needs to be interrupt pin (2 or 3) //orange square cable
#define ENCODER_B 8 // ChB //yellow square cable
#define iCPR 2000 // Counts Per Revolution
volatile int vEncoderValue = 0;
float fAngle = 0;
float fAngle_measurement = 0;

//wheels encoder
#define WHEEL_ENCODER_H1 3 //H1  // pin might change, must be digital, must be interrupt (2 or 3) //white round cable
#define WHEEL_ENCODER_H2 9 //H2  // pin might change, must be digital //blue round cable
volatile int vWheelEncoderValue = 0;
volatile int vOldWheelEncoderValue = 0;
volatile int vWheelTurns = 0;
volatile float fPos_measurement = 0;
int iWheelEncoderBias;

//pendulum encoder func
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

//deprecated
// //correct bias of wheel encoder
// void bias_correct() {
//   int iH1 = digitalRead(WHEEL_ENCODER_H1);
//   int iH2 = digitalRead(WHEEL_ENCODER_H2);
 
//   if ((iH1 == LOW) && (iH2 == LOW)){
//     iWheelEncoderBias = 0;
//   } else if ((iH1 == LOW) && (iH2 == HIGH)){
//     iWheelEncoderBias = 1;
//   } else if ((iH1 == HIGH) && (iH2 == HIGH)){
//     iWheelEncoderBias = 2;
//   } else if ((iH1 == HIGH) && (iH2 == LOW)){
//     iWheelEncoderBias = 3;
//   } else {
//     Serial.println("anti-bias encoder error");
//   }
// }
 
// Define matrix sizes
const int iA_ROWS = 4, iA_COLS = 4;
const int iB_ROWS = 4, iB_COLS = 1;
const int iC_ROWS = 2, iC_COLS = 4;
const int iK_ROWS = 1, iK_COLS = 4;
const int iL_ROWS = 4, iL_COLS = 2;
 
// Define system matrices manually
float mA[iA_ROWS][iA_COLS] = {{0.0, 1.0, 0.0, 0.0},
                                {0., -0.01923077, -0.94326923, 0.0},
                                {0.0, 0.0, 0.0, 1.0},
                                {0., -0.03205128, 12.92211538, -0.60897436}};
 
float mB[iB_ROWS][iB_COLS] = {{0},
                                {1.92307692},
                                {0},
                                {-3.20512821}};
 
float mC[iC_ROWS][iC_COLS] = {{1, 0, 0, 0},
                                {0, 0, 1, 0}};
 
float mK_pole[iK_ROWS][iK_COLS] = {{0.011, 0.18, 15.13, 2.69}};
 
float mL[iL_ROWS][iL_COLS] = {{1.24950830e+02, -1.09396066e+00},
                                {2.49684861e+03, -2.82712481e+01},
                                {-1.12594830e+00, 8.44209651e+01},
                                {-3.06665861e+01, 1.46729546e+03}};
 
// Global variables
// float fAlpha = 0.3;
float fAlpha = 0.6;
float fPreviousU = 0;
float fU = 0;
float fM = 0.05;
float fMassM = 0.52;
float fL = 0.6;
float fD = 0.01;
float fC = 0.01;
float fG = 9.81;
float fTargetX = 1;
float fRWheel = 0.065;
float fKv = 1.673;
float fKt = 0.0892;
float fResistance = 5;
float fVmax = 12;
float fDt = 0.008;
float fEpsilon = 0.01;
float arrX[4] = {0, 0, 0, 0};
float arrX_hat[4] = {0, 0, 0, 0};
float arrY[2] = {0, 0};

// Function to perform matrix multiplication
void multiplyMatrixVector(float mMatrix[][4], float arrVector[], float arrResult[], int iRows, int iCols) {
    for (int i = 0; i < iRows; i++) {
        arrResult[i] = 0;
        for (int j = 0; j < iCols; j++) {
            arrResult[i] += mMatrix[i][j] * arrVector[j];
        }
    }
}
 
// Clamping function
float clamp(float fValue, float fMinVal, float fMaxVal) {
    if (fValue < fMinVal) return fMinVal;
    if (fValue > fMaxVal) return fMaxVal;
    return fValue;
}
 
// Control function
int polePlacementController(float arrX[]) {
    float fAngleDeg = arrX[2]; // 状态中 arrX[2] 是角度（弧度），转换为度
    // float thresholdDeg = 0.3;               // 阈值（0.5度）

    float thresholdDeg = 0.2;               // 阈值（0.5度）

    // 如果角度很小，不输出控制力
    if (abs(fAngleDeg) < thresholdDeg) {
        fU = 0;
        fPreviousU = 0;
        return 0; // 停止输出 PWM
    }

    // 正常控制逻辑
    float fTempU = 0;
    for (int i = 0; i < iK_COLS; i++) {
        fTempU += mK_pole[0][i] * arrX[i];
    }
    fTempU = fTempU / 20;
    fU = clamp(fTempU, -10.0, 10.0);

    float fUSmoothed = fAlpha * fPreviousU + (1 - fAlpha) * fU;
    fPreviousU = fUSmoothed;

    float fTau = fRWheel * fUSmoothed;
    float fI = fTau / fKt;
    float fV = fResistance * fI;
    float fD = fV / fVmax;
    int iPwm = (int)(fD * 800);

    iPwm = clamp(iPwm, -800, 800);
    return iPwm;
}

 
// state_estimator function
void state_estimator(float arrX_hat[], float arrY[], float fU) {
    float arrY_hat[2] = {0};
    multiplyMatrixVector(mC, arrX_hat, arrY_hat, iC_ROWS, iC_COLS);
   
    float arrX_hat_dot[4] = {0};
    multiplyMatrixVector(mA, arrX_hat, arrX_hat_dot, iA_ROWS, iA_COLS);
   
    for (int i = 0; i < iB_ROWS; i++) {
        arrX_hat_dot[i] += mB[i][0] * fU;
    }
   
    float arrCorrection[4] = {0};
    for (int i = 0; i < iL_ROWS; i++) {
        for (int j = 0; j < iC_ROWS; j++) {
            arrCorrection[i] += mL[i][j] * (arrY[j] - arrY_hat[j]);
        }
    }
   
    for (int i = 0; i < 4; i++) {
        arrX_hat[i] += fDt * (arrX_hat_dot[i] + arrCorrection[i]);
        Serial.print("arrX_hat:");
        Serial.println(arrX_hat[i]);
    }
}
 
void Step() {
  arrY[0] = fPos_measurement / 1000; //pos in mm
  arrY[1] = fAngle_measurement;
 
  state_estimator(arrX_hat, arrY, fU);
 
  iPwm = polePlacementController(arrX_hat);
 
  // Send PWM signal to motor
  mc.setSpeed(1, iPwm);
  mc.setSpeed(2, iPwm);
}
 
void display() { //display information of the robot
  Serial.print("W:");
  Serial.print(fPos_measurement / 1000);
 
  Serial.print("|A:");
  Serial.print(fAngle_measurement);
 
  Serial.print("|u:");
  Serial.print(fU);
  Serial.print("|p:");
  Serial.println(iPwm);
}
 
void LEDblink() { //blink LED (debugging mostly)
  if (vLed13 == 1){
      digitalWrite(LED,LOW);
      vLed13 = 0;
    } else {
      digitalWrite(LED,HIGH);
      vLed13 = 1;
    }
}
 
void PendulumAngleMeasure() {
  // Calculate degrees of rotation
    fAngle = (float(vEncoderValue) / iCPR) * 360.0;
    fAngle_measurement = fAngle;
 
    // Handle wrap-around for angle (0 to 360 degrees)
    if (fAngle < 0) {
      fAngle += 360;
    } else if (fAngle >= 360) {
      fAngle -= 360;
    }
 
    if (fAngle_measurement > 180){
      fAngle_measurement -= 360;
    }
}
 
void wheel_encoder_voltage() {
  //measure
  int iH1 = digitalRead(WHEEL_ENCODER_H1);
  int iH2 = digitalRead(WHEEL_ENCODER_H2);
 
  // find corresponding angle
  if ((iH1 == LOW) && (iH2 == LOW)){
    vWheelEncoderValue = 0;
  } else if ((iH1 == LOW) && (iH2 == HIGH)){
    vWheelEncoderValue = 1;
  } else if ((iH1 == HIGH) && (iH2 == HIGH)){
    vWheelEncoderValue = 2;
  } else if ((iH1 == HIGH) && (iH2 == LOW)){
    vWheelEncoderValue = 3;
  } else {
    Serial.println("wheel encoder error");
  }
 
  //check if extra full rotation
  if (vOldWheelEncoderValue != vWheelEncoderValue){
    if ((vWheelEncoderValue == 0) && (vOldWheelEncoderValue == 3)){
      vWheelTurns++;
    } else if ((vWheelEncoderValue == 0) && (vOldWheelEncoderValue == 2)){
      vWheelTurns++;
    } else if ((vWheelEncoderValue == 3) && (vOldWheelEncoderValue == 1)){
      vWheelTurns--;
    } else if ((vWheelEncoderValue == 3) && (vOldWheelEncoderValue == 0)){
      vWheelTurns--;
    }
  }
 
  vOldWheelEncoderValue = vWheelEncoderValue;
  iCounter++;
}
 
void WheelAngleMeasure() {
  // Calculate rotation and thus position
  float fAngleFrac = float(vWheelTurns) / 60; //60 obtained through experimentation
  fPos_measurement = fAngleFrac * 204.2; //3.14*65 == 408.2 (mm travelled for 1 full rotation)
}
 
void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to open
 
  //LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED,LOW);
 
  Serial.println("Starting encoder countdown : 5s");
 
  delay(5000); //5s delay to set pendulum to 0 degree (performance)
 
  //Encoder theta (pendulum)
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), pendulum_encoder_voltage, CHANGE); //interrupt //needs interrupt pin for A
 
  //encoder x (wheels)
  pinMode(WHEEL_ENCODER_H1, INPUT_PULLUP); //do we need pullup ? -> check in lab
  pinMode(WHEEL_ENCODER_H2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WHEEL_ENCODER_H1), wheel_encoder_voltage, CHANGE); //interrupt //needs interrupt pin for H1
 
  //I2C coms with motor shield
  Wire.begin();
  mc.reinitialize();    // Bytes: 0x96 0x74
  mc.disableCrc();      // Bytes: 0x8B 0x04 0x7B 0x43
  mc.clearResetFlag();  // Bytes: 0xA9 0x00 0x04
 
  // By default, the Motoron is configured to stop the motors if
  // it does not get a motor control command for 1500 ms.
  // You can uncomment a line below to adjust this time or disable
  // the timeout feature.
  // mc.setCommandTimeoutMilliseconds(1000);
  // mc.disableCommandTimeout();
 
  // Configure motor 1
  mc.setMaxAcceleration(1, 140);
  mc.setMaxDeceleration(1, 300);
 
  // Configure motor 2
  mc.setMaxAcceleration(2, 200);
  mc.setMaxDeceleration(2, 300);
 
  // Configure motor 3
  mc.setMaxAcceleration(3, 80);
  mc.setMaxDeceleration(3, 300);
 
}
 
void loop() {
 
  ulStartTime = millis();
 
  LEDblink();
  PendulumAngleMeasure();
  WheelAngleMeasure();
  Step();
 
  ulEndTime = millis();
  ulDuration = ulEndTime - ulStartTime;
 
  display();
}


