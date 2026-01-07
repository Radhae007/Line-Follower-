#include <QTRSensors.h>

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// Motor A (Left)
const int MOTOR_A1 = 12;
const int MOTOR_A2 = 11;
const int ENA_PIN  = 10;
// int c=0; // Checkpoint counter commented out

// LEDs - Commented out
// const int led1 = 6;   // Yellow
// const int led2 = 5;   // Green
// const int led3 = 4;   // Red

// Motor B (Right)
const int MOTOR_B1 = 8;
const int MOTOR_B2 = 7;
const int ENB_PIN  = 9;

// ================= PID ======================
#define Kp 0.09
#define Ki 0
#define Kd 0.12
#define BaseSpeed 150
#define MaxSpeed 200

int lastError = 0;
int lastCorr = 0;

// Weights for center = 0
int weights[8] = { -3500, -2500, -1500, -500, 500, 1500, 2500, 3500 };


// =====================================================
//                      SETUP
// =====================================================
void setup() {
  Serial.begin(9600);

  pinMode(ENA_PIN, OUTPUT);
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);

  pinMode(ENB_PIN, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);

  /* LEDs setup commented out
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  */

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, SensorCount);
  qtr.setEmitterPin(3);
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);

  // Calibration
  Serial.println("Calibrating...");
  for (int i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(5);
  }
  Serial.println("Calibration Done!");
}


// =====================================================
//                      LOOP
// =====================================================
void loop() {

  qtr.readCalibrated(sensorValues);

  long weightedSum = 0;
  long total = 0;

  for (int i = 0; i < 8; i++) {
    weightedSum += (long)weights[i] * sensorValues[i];
    total += sensorValues[i];
  }

  if (total == 0) return;

  long error = weightedSum / total;

  if (abs(error) < 40) error = 0;

  int derivative = error - lastError;
  int correction = (Kp * error) + (Kd * derivative);
  lastError = error;

  // Error smoothing
  int delta = correction - lastCorr;
  delta = constrain(delta, -15, 15);
  correction = lastCorr + delta;
  lastCorr = correction;

  int leftSpeed  = constrain(BaseSpeed + correction, -MaxSpeed, MaxSpeed);
  int rightSpeed = constrain(BaseSpeed - correction, -MaxSpeed, MaxSpeed);

  move(leftSpeed, rightSpeed);
}



// =====================================================
//                    MOVEMENT LOGIC
// =====================================================
void move(int leftSpeed, int rightSpeed) {

  int blackCount = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] > 700) blackCount++;
  }

  // =====================================================
  //          ALL BLACK → X / T / CHECKPOINT 
  // =====================================================
  if (blackCount == 8) {
    Straight();
    delay(200); // Just move past the intersection
    return;
    
    /* CHECKPOINT LOGIC COMMENTED OUT
    if (c==0){
       Stop();
       delay(5000);
       c++;
    }
    // ... rest of the logic
    */
  }


  // =====================================================
  //                SHARP TURN DETECTION
  // =====================================================

  // ----------- SHARP LEFT -----------
  if (sensorValues[0] > 700 && sensorValues[1] > 700 && sensorValues[7] < 400) 
  {
    // digitalWrite(led1, HIGH); // LED commented out
    while (sensorValues[3] < 600 && sensorValues[4] < 600) {
      LeftRot();
      qtr.readCalibrated(sensorValues);
    }
    // digitalWrite(led1, LOW); // LED commented out
    return;
  }


  // ----------- SHARP RIGHT -----------
  if (sensorValues[7] > 700 && sensorValues[6] > 700 && sensorValues[0] < 400) 
  {
    // digitalWrite(led2, HIGH); // LED commented out
    while (sensorValues[3] < 600 && sensorValues[4] < 600) {
      RightRot();
      qtr.readCalibrated(sensorValues);
    }
    // digitalWrite(led2, LOW); // LED commented out
    return;
  }


  // =====================================================
  //              NORMAL PID MOTOR CONTROL
  // =====================================================
  if (leftSpeed >= 0) {
    digitalWrite(MOTOR_A1, HIGH);
    digitalWrite(MOTOR_A2, LOW);
    analogWrite(ENA_PIN, leftSpeed);
  } else {
    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, HIGH);
    analogWrite(ENA_PIN, -leftSpeed);
  }

  if (rightSpeed >= 0) {
    digitalWrite(MOTOR_B1, HIGH);
    digitalWrite(MOTOR_B2, LOW);
    analogWrite(ENB_PIN, rightSpeed);
  } else {
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, HIGH);
    analogWrite(ENB_PIN, -rightSpeed);
  }
}



// =====================================================
//                  MOTOR FUNCTIONS
// =====================================================
void Stop() {
  analogWrite(ENA_PIN, 0);
  analogWrite(ENB_PIN, 0);
}

void LeftRot() {
  analogWrite(ENA_PIN, 140);
  analogWrite(ENB_PIN, 140);
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, HIGH);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
}

void RightRot() {
  analogWrite(ENA_PIN, 140);
  analogWrite(ENB_PIN, 140);
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, HIGH);
}

void Straight() {
  analogWrite(ENA_PIN, BaseSpeed);
  analogWrite(ENB_PIN, BaseSpeed);
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
}
