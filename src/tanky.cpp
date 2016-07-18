#include <Arduino.h>

//Libraries
#include <Servo.h>
#include <MPU6050.h>
#include <I2Cdev.h>
#include <Wire.h>

//Define parameters for the pan servo - straight ahead is not dead on 90 degrees
#define PAN_MID 83
#define PAN_RIGHT 0
#define PAN_LEFT 179

//Define parameters for tilt servo
#define TILT_MID 160
#define TILT_DOWN 179
#define TILT_LEFT 100

#define IR_PIN A8									//Pin that the IR sensor is connected to
#define PING_PIN 3								//Pin that the ultrasonic sensor is connected to
#define SWITCH_ONE 2							//Interrupt pin that the bump sensors are connected to
#define LED_PIN 13								//Pin that the LED is connected to

#define firstMotorSpeed  6 				//Speed control
#define secMotorSpeed  5

#define firstMotorDir  7 					//Direction control
#define secMotorDir  4

int degreeGap = 6;								//Define the size of gaps that the robot can fit through
volatile boolean goBack = false;	//This tells the robot whether it needs to back out of a situation

Servo pan, tilt;
int turnDegree = 20;
MPU6050 accel;
int16_t ax, ay, az;

//Servo rotates 180 degrees, so there are 180 readings
float mapValues [180];
float mapValuesOrdered [180];

//Declare functions
void go(int duration, int pwm);
void crashed();
void scan();
int findBestGap();
float getMedian();
void smoothData();
float getGapSize(float sideA, float sideB, float angle);
void checkForObstacle();
int getForwardDistance();
void getOutOfThere();
void checkAccel();
void goBackward(int duration, int pwm);
void rotateRight(int duration, int pwm);
void rotateLeft(int duration, int pwm);
void copyArray();

void setup() {

  Serial.begin(9600);

  delay(2500);
  Wire.begin();
  pan.attach(9);
  tilt.attach(10);
  accel.initialize();
	
  //Set up interrupt on pin 2, with crashed() as the ISR
  digitalWrite(SWITCH_ONE, HIGH);
  attachInterrupt(0, crashed, CHANGE);

  pinMode(firstMotorDir, OUTPUT);
  pinMode(secMotorDir, OUTPUT);
  pinMode(PING_PIN, OUTPUT);

  //Tell servos to move to center positions
  pan.write(PAN_MID);
  delay(100);
  tilt.write(TILT_MID);
  delay(2500);

  Serial.println("Setup complete");
}

void loop() {

  go(30000, 255);
  delay(100);

}

void go(int duration, int pwm) {

  long a,b;
  boolean shouldMove = true;
  a = millis();

  do {

    checkForObstacle();
    checkAccel();

    if(goBack == false) {
      digitalWrite(firstMotorDir, HIGH);
      digitalWrite(secMotorDir, HIGH);
      analogWrite(firstMotorSpeed, pwm);
      analogWrite(secMotorSpeed, pwm);
    }
    else {
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
      goBack = false;
    }

    b = millis()-a;
    if(b>=duration) {
      shouldMove=false;
    }
  }
  while(shouldMove!=false);

  analogWrite(firstMotorSpeed, 0);
  analogWrite(secMotorSpeed, 0);
}

void crashed() {

  goBack = true;
}

void scan() {

  int a = 0;

  pan.write(PAN_RIGHT);
  delay(50);
  for(int i=PAN_RIGHT; i<=PAN_LEFT; i++) {
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
  delay(50);
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
      startEdge = i;
      if(i==0) {
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

  for(c=1; c<180; c++) {
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

  float total,newMean;

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

  for(int i=0; i<=50; i++) {
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

void goBackward(int duration, int pwm) {

  //Go backwards
  digitalWrite(firstMotorDir, LOW); 	
  digitalWrite(secMotorDir, LOW); 
	
	//Set speed	
  analogWrite(firstMotorSpeed, pwm); 	
  analogWrite(secMotorSpeed, pwm);

	//Wait for the right amount of time
  delay(duration);

	//Stop motors
  analogWrite(firstMotorSpeed, 0); 
  analogWrite(secMotorSpeed, 0);
}

void rotateRight(int duration, int pwm) {

	//Left motor goes forward
  digitalWrite(firstMotorDir, HIGH); 	

	//Right motor goes backwards
  digitalWrite(secMotorDir, LOW); 	

	//Set speed
  analogWrite(firstMotorSpeed, pwm); 	
  analogWrite(secMotorSpeed, pwm);

	//Wait for the right amount of time
  delay(duration);

	//Stop motors
  analogWrite(firstMotorSpeed, 0); 	
  analogWrite(secMotorSpeed, 0);
}

void rotateLeft(int duration, int pwm) {

	//Left motor goes backwards
  digitalWrite(firstMotorDir, LOW);
 	
	//Right motor goes forwards
  digitalWrite(secMotorDir, HIGH);
 	
	//Set speed
  analogWrite(firstMotorSpeed, pwm); 	
  analogWrite(secMotorSpeed, pwm);
	
	//Wait for the right amount of time
  delay(duration);

	//Stop motors
  analogWrite(firstMotorSpeed, 0); 
  analogWrite(secMotorSpeed, 0);
}
