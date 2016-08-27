#include <Arduino.h>

#include <Servo.h>
#include <MPU6050.h>
#include <I2Cdev.h>
#include <Wire.h>
#include <AFMotor.h>

#define PAN_MID 83
#define PAN_RIGHT 0
#define PAN_LEFT 179

#define TILT_MID 160
#define TILT_DOWN 179
#define TILT_LEFT 100

#define IR_PIN A8
#define PING_PIN 15
#define SWITCH_ONE 18
#define LED_PIN 17

AF_DCMotor rightMotor(1, MOTOR12_64KHZ);
AF_DCMotor leftMotor(2, MOTOR12_64KHZ);

int degreeGap = 6;
volatile boolean goBack = false;

Servo pan, tilt;
int turnDegree = 20;
MPU6050 accel;
int16_t ax, ay, az;

float mapValues [180];
float mapValuesOrdered [180];

void setup() { 

  Serial.begin(9600);

  delay(2500);
  Wire.begin();
  pan.attach(10);
  tilt.attach(9);
  accel.initialize();

  digitalWrite(SWITCH_ONE, HIGH);
  attachInterrupt(digitalPinToInterrupt(SWITCH_ONE), crashed, CHANGE);

  pinMode(PING_PIN, OUTPUT);

  pan.write(PAN_MID);
  delay(500);
  tilt.write(TILT_MID);
  delay(500); 

  Serial.println("Setup complete");
} 

void loop() { 

  go(30000, 255);
  delay(100);

}

void go(int duration, int pwm) { 

  long a,b;
  int accelVal, ledVal = 0;
  boolean shouldMove = true;
  a = millis();

  do {
    
    checkForObstacle();
    checkAccel();

    if(goBack == false) {
      rightMotor.setSpeed(255);
      rightMotor.run(FORWARD);
      leftMotor.setSpeed(255);
      leftMotor.run(FORWARD);
    }
    else {    
      goBack = false;
      getOutOfThere();

      int turnTo = findBestGap();

      int actualTurn = turnTo-90;
      int amountToTurn = abs(actualTurn);

      if(turnTo > 90) {
        rotateLeft(turnDegree*amountToTurn, 255);
      }
      else if(turnTo < 90) {
        rotateRight(turnDegree*amountToTurn, 255);
      }
    }

    b = millis()-a;
    if(b>=duration) {
      shouldMove=false;
    }
  } 
  while(shouldMove!=false);

  stopMoving();
} 

void crashed() {

  goBack = true;
}

void scan() {

  int a = 0;

  pan.write(PAN_RIGHT);
  delay(200);
  for(int i=PAN_RIGHT;i<=PAN_LEFT;i++) {
    pan.write(i);
    delay(15);
    int dist = getForwardDistance();
    if(dist > 400) {
      dist = 15;
    }
    if(dist > 300 && dist <=400) {
      dist = 300;
    }
    mapValues[a] = dist; 
    a++;
  }
  pan.write(PAN_MID);
  delay(250);
}

int findBestGap() {

  boolean gapFound = false;
  float median;
  float gapSize;
  float startEdge, endEdge;
  float startDist, endDist;
  int bestStartEdge, bestEndEdge;
  float bestAngle;
  float bestMin = 0;
  int counter;
  int j,i;

  scan();
  median = getMedian();
  smoothData();

  for(i=0;i<180;) {
    j = i+1;

    if(mapValues[i]-median > 0) {
      counter = 1;
      startEdge=i;
      if(i=0) {
        startDist = median;
      }
      else {
        startDist = mapValues[i+1];
      }    
      while(mapValues[j]-median > 0 && j<180) {
        j++;
        counter++;
      }
      endEdge=j+1;
      if(j==179) {
        endDist = median;
      }
      else {
        endDist = mapValues[j];
      }
      if(counter>degreeGap) {
        float minDist = 0;
        gapSize = getGapSize(startDist, endDist, counter);
        if(gapSize > 25) {
          gapFound = true;
          for(int a=startEdge;a<endEdge;a++) {
            if(mapValues[a] > minDist) {
              minDist = mapValues[i];
            }
          }
          if(minDist > bestMin) {
            bestMin = minDist;
            bestStartEdge = startEdge; 
            bestEndEdge = endEdge;
          }
        }
      }
    }

    i=j;
  }

  bestAngle = (bestEndEdge+bestStartEdge)/2;
  int bestToInt = int(bestAngle);

  if(gapFound == true) {
    return bestToInt;
  }
  else {
    getOutOfThere();
    return 90;
  }
}

float getMedian() {

  float median;
  int c, d, t;
  
  copyArray();
  
  for(c=1;c<180;c++) {
    d = c;
    while (d > 0 && mapValuesOrdered[d] < mapValuesOrdered[d-1]) {
      t = mapValuesOrdered[d];
      mapValuesOrdered[d] = mapValuesOrdered[d-1];
      mapValuesOrdered[d-1] = t;
      d--;
    }
  }
  
  median = mapValuesOrdered[89];
  return median;
}

void copyArray() {
  
  for(int i=0;i<180;i++) {
    mapValuesOrdered[i] = mapValues[i];
  }
  
}

void smoothData() {

  float total, newMean;

  for(int i=0;i<178;i++) {
    newMean = 0;
    total = mapValues[i];
    for(int j=i+1;j<=i+2;j++) {
      total += mapValues[j];
    }
    newMean = total/3;

    mapValues[i] = newMean;
  }

}

float getGapSize(float sideA, float sideB, float angle) {

  float degToRadians;
  float gapSquared, gap;
  float angleCos;

  degToRadians = (angle*PI)/180;
  angleCos = cos(degToRadians);

  gapSquared = ((sideA*sideA)+(sideB*sideB)) - ((2*sideA*sideB)*angleCos); 
  gap = sqrt(gapSquared);

  return gap; 
}

void checkForObstacle() {

  float num_readings = 10;
  float total = 0;
  float distance;
  float average;
  float cmDistance;

  for(int i=0;i<num_readings;i++) {
    distance = 1234.85*pow(analogRead(IR_PIN),-1.15);
    total += distance;
    delay(5);
  }

  average = total/num_readings;
  
  cmDistance = average*10;

  Serial.println(cmDistance);

  if(cmDistance < 20) {
    goBack = true;
  }
}

int getForwardDistance() {

  int distance;
  unsigned long pulseDuration = 0;

  pinMode(PING_PIN, OUTPUT);
  digitalWrite(PING_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(PING_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(PING_PIN, LOW);

  pinMode(PING_PIN, INPUT);
  pulseDuration = pulseIn(PING_PIN,HIGH);
  pulseDuration = pulseDuration/2;

  distance = int(pulseDuration/29);
  return distance;
} 

void getOutOfThere() {

  int randDir, randTime;
  randTime = random(500,3001);
  randDir = random(1, 3);
  
  analogWrite(LED_PIN, 255);
  goBackward(1000, 255);

  if(randDir == 1) {
    rotateLeft(randTime, 255);
  }
  else {
    rotateRight(randTime, 255);
  }

  analogWrite(LED_PIN, 0);
}

void checkAccel() {
  
  float totalX = 0;
  float totalY = 0;
  float averageX, averageY;
  
  for(int i=0;i<=50;i++) {
    accel.getAcceleration(&ax, &ay, &az);
    totalX += ax;
    totalY += ay;
  }
  averageX = totalX/50;
  averageY = totalY/50;
  
  if(averageX < -9000) {
    goBack = true;
  }
  else if (averageY > 5000 || averageY < -5000) {
    goBack = true;
  }
}


void stopMoving() {

  rightMotor.run(RELEASE);
  leftMotor.run(RELEASE);
}

void goBackward(int duration, int pwm) { 

  rightMotor.setSpeed(pwm);
  rightMotor.run(BACKWARD);
  leftMotor.setSpeed(pwm);
  leftMotor.run(BACKWARD);
  delay(duration);
  stopMoving();
} 

void rotateRight(int duration, int pwm) { 

  rightMotor.setSpeed(pwm);
  rightMotor.run(BACKWARD);
  leftMotor.setSpeed(pwm);
  leftMotor.run(FORWARD);
  delay(duration);
  stopMoving();
} 

void rotateLeft(int duration, int pwm) { 

  rightMotor.setSpeed(pwm);
  rightMotor.run(FORWARD);
  leftMotor.setSpeed(pwm);
  leftMotor.run(BACKWARD);
  delay(duration);
  stopMoving();
} 








