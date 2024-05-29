#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "dataArray.h"
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int servonum = 12;
int randnumber;
const float servodeg0[12] = {90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90};
float servodegnew[servonum]; 
float servodegold[servonum]; 
const int servodir[12] = {+1, +1, -1, -1, -1, +1, -1, -1, -1, +1, +1, +1}; 
const float pi = 3.1416;
const float alfa0 = pi / 6; 
const float beta0 = pi / 3; 
const float jointlength = 50;
const float width = 120; 
const float leng = 120; 
const float distag = 12; 
const float toeout0 = 20; 
const float leglength0 = 2 * jointlength * cos(alfa0);
const float gamma0 = asin(toeout0 / (leglength0 + distag)); 
const float bodyradius = sqrt(pow((width / 2), 2) + pow((leng / 2), 2)); 
const float phi0 = atan(width / leng); 
const float height0 = sqrt(pow(leglength0 + distag, 2) - pow(toeout0, 2)); 
float leglength [4] = {sqrt(pow(height0, 2) + pow(toeout0, 2)), sqrt(pow(height0, 2) + pow(toeout0, 2)),
                       sqrt(pow(height0, 2) + pow(toeout0, 2)), sqrt(pow(height0, 2) + pow(toeout0, 2))
                      };

unsigned long timestep = 500; 
int steplength = 70; 
float phi = 20; 

float footpos[12]; 
float stepturn[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

const float jointangle0[12] = {alfa0, beta0, 0, alfa0, beta0, 0, alfa0, beta0, 0, alfa0, beta0, 0};
float jointangle[12]; 
#define SERVOMIN  150  
#define SERVOMAX  600  
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60); 
  delay(10);

  for (int i = 0; i < servonum; i++) {    
    servodegnew[i] = servodeg0[i]; 
    servodegold[i] = servodegnew[i];
    pwm.setPWM(i, 0, map(servodegnew[i], 0, 180, SERVOMIN, SERVOMAX));
  } 

  delay(7000);
}

void loop() {

  for (int j =0; j<= arrayLength; j++ ){
    if (arr[j]==1){
      Serial.println("Moving down");
      forwardcreep();
      delay(500);
      forwardcreep();
      delay(500);
      forwardcreep();
      delay(500);
      forwardcreep();
      delay(500);
      forwardcreep();
      delay(500);
      forwardcreep();
      delay(500);
      forwardcreep();
      delay(500);
      forwardcreep();
      delay(1000);
    }
    if (arr[j]==2){
      Serial.println("Moving left");
      leftturn90deg();
      delay(500);
      forwardcreep();
      delay(500);
      forwardcreep();
      delay(500);
      forwardcreep();
      delay(500);
      forwardcreep();
      delay(500);
      forwardcreep();
      delay(500);
      forwardcreep();
      delay(500);
      forwardcreep();
      delay(500);
      forwardcreep();
      delay(500);
      forwardcreep();
      delay(500);
      rightturn90deg();
      delay(1000);
    }
    if (arr[j]==0){
      Serial.println("Moving right");
      rightturn90deg();
      delay(500);
      forwardcreep();
      delay(500);
      forwardcreep();
      delay(500);
      forwardcreep();
      delay(500);
      forwardcreep();
      delay(500);
      forwardcreep();
      delay(500);
      forwardcreep();
      delay(500);
      forwardcreep();
      delay(500);
      forwardcreep();
      delay(500);
      forwardcreep();
      delay(500);
      leftturn90deg();
      delay(1000);
    }
    if (arr[j]==3){
      Serial.println("Moving up");
      reversecreep();
      delay(500);
      reversecreep();
      delay(500);
      reversecreep();
      delay(1000);
    }
  } 
  moveStop();
  delay(10000);
}

void moveStop() {

  for (int i = 0; i < servonum; i++) {
    int pulseLength = angleToPulse(90); 
    pwm.setPWM(i, 0, pulseLength);
  }
}

void lengthangles() {

  jointangle[2] = gammaleft(footpos[1], footpos[2]);
  leglength[0] = legleft(footpos[0], footpos[2], jointangle[2]);
  jointangle[1] = beta(leglength[0]);
  jointangle[0] = alfafront(footpos[0], jointangle[1], leglength[0]);

  jointangle[5] = gammaleft(footpos[4], footpos[5]);
  leglength[1] = legleft(footpos[3], footpos[5], jointangle[5]);
  jointangle[4] = beta(leglength[1]);
  jointangle[3] = alfarear(footpos[3], jointangle[4], leglength[1]);

  jointangle[8] = gammaright(footpos[7], footpos[8]);
  leglength[2] = legright(footpos[6], footpos[8], jointangle[8]);
  jointangle[7] = beta(leglength[2]);
  jointangle[6] = alfafront(footpos[6], jointangle[7], leglength[2]);

  jointangle[11] = gammaright(footpos[10], footpos[11]);
  leglength[3] = legright(footpos[9], footpos[11], jointangle[11]);
  jointangle[10] = beta(leglength[3]);
  jointangle[9] = alfarear(footpos[9], jointangle[10], leglength[3]);
}

float gammaleft (float dy, float dz) {
  float gresult = atan((toeout0 + dy) / (height0 - dz)) - gamma0;
  return gresult;
}

float gammaright(float dy, float dz) {
  float gresult = gamma0 - atan((toeout0 - dy) / (height0 - dz));
  return gresult;
}

float legleft(float dx, float dz, float gamma) {
  float lresult = sqrt(pow(leglength0 - (dz / cos(gamma0 + gamma)), 2) + pow(dx, 2));
  if (lresult > 2 * jointlength) lresult = 2 * jointlength;
  return lresult;
}

float legright(float dx, float dz, float gamma) {
  float lresult = sqrt(pow(leglength0 - (dz / cos(gamma0 - gamma)), 2) + pow(dx, 2));
  if (lresult > 2 * jointlength) lresult = 2 * jointlength;
  return lresult;
}

float beta(float leg) {
  float bresult = 2 * acos(leg / (2 * jointlength));
  return bresult;
}

float alfafront(float dx, float beta, float leg) {
  float aresult = (beta / 2) - asin(dx / leg);
  return aresult;
}

float alfarear(float dx, float beta, float leg) {
  float aresult = (beta / 2) + asin(dx / leg);
  return aresult;
}

void turnpos(float f) {
  stepturn[0] = bodyradius * cos(phi0 + (f * pi / 180)) - leng / 2;
  stepturn[1] = bodyradius * sin(phi0 + (f * pi / 180)) - width / 2;
  stepturn[3] = bodyradius * cos(pi - phi0 + (f * pi / 180)) + leng / 2;
  stepturn[4] = bodyradius * sin(pi - phi0 + (f * pi / 180)) - width / 2;
  stepturn[6] = bodyradius * cos(2 * pi - phi0 + (f * pi / 180)) - leng / 2;
  stepturn[7] = bodyradius * sin(2 * pi - phi0 + (f * pi / 180)) + width / 2;
  stepturn[9] = bodyradius * cos(pi + phi0 + (f * pi / 180)) + leng / 2;
  stepturn[10] = bodyradius * sin(pi + phi0 + (f * pi / 180)) + width / 2;
}

void servopos() {
  for (int i = 0; i < 12; i++) {
    servodegnew[i] = servodeg0[i] + servodir[i] * (jointangle[i] - jointangle0[i]) * 180 / pi;
  }
}

void servomove() {
  int servotimeold[servonum]; 
  int servotimenew[servonum]; 
  int SERVOPULSE[servonum]; 
  float servodeg[servonum]; 
  float servodegspeed[servonum]; 
  unsigned long starttime = millis(); 
  unsigned long timenow = starttime; 
  for (int i = 0; i < servonum; i++) {
    servodegspeed[i] = (servodegnew[i] - servodegold[i]) / timestep; 
    servodeg[i] = servodegold[i]; 
    servotimeold[i] = starttime; 
  }
  while ((timenow - starttime) < timestep) {
    for (int i = 0; i < servonum; i++) { 
      servotimenew[i] = millis(); 
      servodeg[i] += servodegspeed[i] * (servotimenew[i] - servotimeold[i]);

      int pulseLength = angleToPulse(servodeg[i]); 
      pwm.setPWM(i, 0, pulseLength); 
      servotimeold[i] = servotimenew[i];
    }
    timenow = millis();
    
  }
  for (int i = 0; i < servonum; i++) { 
    int pulseLength = angleToPulse(servodeg[i]); 
    pwm.setPWM(i, 0, pulseLength); 
    servotimeold[i] = servotimenew[i];
  }
}

void servomovefast() {
  for (int i = 0; i < servonum; i++) { 
    int pulseLength = angleToPulse(servodegnew[i]); 
    pwm.setPWM(i, 0, pulseLength); 
    servodegold[i] = servodegnew[i]; 
  }
  delay(100); 
}

void footxyz(int i, float x, float y, float z) {
  footpos[3 * i] = x;
  footpos[3 * i + 1] = y;
  footpos[3 * i + 2] = z;
  lengthangles();
  servopos();
}

void footmovexyz(int i, float x, float y, float z) {
  footpos[3 * i] += x;
  footpos[3 * i + 1] += y;
  footpos[3 * i + 2] += z;
  lengthangles();
  servopos();
}

void bodyxyz(float x, float y, float z ) {
  
  for (int i = 0; i < 4; i++) {
    footpos[3 * i] = -x;
    footpos[3 * i + 1] = -y;
    footpos[3 * i + 2] = -z;
  }
  lengthangles();
  servopos();
}

void bodymovexyz(float x, float y, float z ) {
  
  for (int i = 0; i < 4; i++) {
    footpos[3 * i] -= x;
    footpos[3 * i + 1] -= y;
    footpos[3 * i + 2] -= z;
  }
  lengthangles();
  servopos();
}

void bodytwist(float f) {
  turnpos(-f);
  for (int i = 0; i < 12; i++) {
    footpos[i] += stepturn[i];
  }
  lengthangles();
  servopos();
}

void footstep (int i, float x, float y) {
  footmovexyz(i, 0, 0, 30);
  servomovefast();
  footmovexyz(i, x, y, 0);
  servomovefast();
  footmovexyz(i, 0, 0, -30);
  servomovefast();
}

void (footstepangle(int i, float f)) {
  turnpos(f);
  footmovexyz(i, 0, 0, 30);
  servomovefast();
  footmovexyz(i, stepturn[3 * i], stepturn [3 * i + 1], 0);
  servomovefast();
  footmovexyz(i, 0, 0, -30);
  servomovefast();
}

void forwardcreep() {
  bodymovexyz(steplength / 4, -toeout0, 0);
  servomove();
  footstep(1, steplength / 2, 0);
  footstep(0, steplength / 2, 0); 
  bodymovexyz(steplength / 4, 2 * toeout0, 0); 
  servomove();
 {
    footstep(3, steplength, 0); 
    footstep(2, steplength, 0); 
    bodymovexyz(steplength / 2, -2 * toeout0, 0);
    servomove();
    footstep(1, steplength, 0);
    footstep(0, steplength, 0);
    bodymovexyz(steplength / 2, 2 * toeout0, 0);
    servomove();
 }
  footstep(3, steplength / 2, 0);
  footstep(2, steplength / 2, 0);
  bodyxyz(0, 0, 0); 
  servomove();
}

void reversecreep() {
  bodymovexyz(-steplength / 4, -toeout0, 0);
  servomove();
  footstep(0, -steplength / 2, 0);
  footstep(1, -steplength / 2, 0);
  bodymovexyz(-steplength / 4, 2 * toeout0, 0);
  servomove();
{
    footstep(2, -steplength, 0);
    footstep(3, -steplength, 0);
    bodymovexyz(-steplength / 2, -2 * toeout0, 0);
    servomove();
    footstep(0, -steplength, 0);
    footstep(1, -steplength, 0);
    bodymovexyz(-steplength / 2, 2 * toeout0, 0);
    servomove();
  }
  footstep(2, -steplength / 2, 0);
  footstep(3, -steplength / 2, 0);
  bodyxyz(0, 0, 0);
  servomove();
}

void leftturn() {
{
    bodyxyz(toeout0 / 2, toeout0, 0);
    servomove();
    footstepangle(3, phi);
    footstepangle(2, phi);
    footxyz(0, -toeout0 / 2 - stepturn[0], toeout0 - stepturn[1], 0);
    footxyz(1, -toeout0 / 2 - stepturn[3], toeout0 - stepturn[4], 0);
    footxyz(2, -toeout0 / 2, toeout0, 0);
    footxyz(3, -toeout0 / 2, toeout0, 0);
    servomove();
    footstepangle(0, phi);
    footstepangle(1, phi);
  }
  bodyxyz(0, 0, 0);
  servomove();
}

void leftturn90deg(){
  for (int i = 0; i < 10; i++) {
    leftturn();
  }
}

void rightturn() {
{
    bodyxyz(-toeout0 / 2, toeout0, 0);
    servomove();
    footstepangle(2, -phi);
    footstepangle(3, -phi);
    footxyz(0, toeout0 / 2 - stepturn[0], toeout0 - stepturn[1], 0);
    footxyz(1, toeout0 / 2 - stepturn[3], toeout0 - stepturn[4], 0);
    footxyz(2, toeout0 / 2, toeout0, 0);
    footxyz(3, toeout0 / 2, toeout0, 0);
    servomove();
    footstepangle(1, -phi); 
    footstepangle(0, -phi);
  }
  bodyxyz(0, 0, 0);
  servomove();
}

void rightturn90deg(){
  for (int i = 0; i < 9; i++) { 
    rightturn();
  }
}
