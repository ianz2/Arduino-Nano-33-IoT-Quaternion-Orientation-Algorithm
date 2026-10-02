#include <Arduino_LSM6DS3.h>
#include "Quaternion.h"
#define PI 3.1415926535897932384626433832795

// === Variables === --------------------------------------------------------------------------------------------------------------------------------

float x, y, z, rnx, rny, rnz, rdx, rdy, rdz;

double delta = 0.02;
double correction = 0.1;

unsigned long lastRunTime = micros();
unsigned long nowTime = micros();

double qw, qx, qy, qz;
double qw0 = 1;
double qx0 = 0;
double qy0 = 0;
double qz0 = 0;

double pitch;
double roll;
double yaw;

const int input1 = 2;
const int input2 = 3;
const int speedPin = 5;
const int ledPin = 13;

Quaternion q0 = Quaternion(qw0, qx0, qy0, qz0);
Quaternion down = Quaternion(0, 0, 0, -1);
Quaternion straight = Quaternion(0, 1, 0, 0);
Quaternion qgEstimate = Quaternion(1, 0, 0, 0);
Quaternion qsEstimate = Quaternion(1, 0, 0, 0);

double crossProduct2d(double a, double b, double c, double d){
  return(a*d-c*b);
}

double mag(double a, double b, double c) {
  double m = sqrt(a*a + b*b + c*c);
  return(m);
}

void vectorPrint(double a, double b, double c){
  Serial.print("Vector: <");
  Serial.print(a);
  Serial.print(", ");
  Serial.print(b);
  Serial.print(", ");
  Serial.print(c);
  Serial.println(">");
}


void setup() {

  delay(5000);
  Serial.begin(9600);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  pinMode(input1, OUTPUT);
  pinMode(input2, OUTPUT);
  pinMode(speedPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

}

void loop() {

  // --- Accelerometer ---
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    
    /*Serial.print("Acceleration: ");
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.print(z);
    Serial.println('\t');
    */

    double xx = (double)x;
    double yy = (double)y;
    double zz = (double)z;
    double magAccel = mag(xx, yy, zz);

    // --- Gravity Orientation Normalization ---
    if(magAccel >= (1-delta) && magAccel <= (1+delta) && (abs(rdx) < 1 && abs(rdy) < 1 && abs(rdz) < 1)){

      if(abs(x) < delta){
        x = 0;
      }
      if(abs(y) < delta){
        y = 0;
      }
      if(abs(z) < delta){
        z = 0;
      }

      //normalize x, y, z
      x = (x/magAccel);
      y = (y/magAccel);
      z = (z/magAccel);

      rnx = x-qgEstimate.getX();
      rny = y-qgEstimate.getY();
      rnz = -1*(z-qgEstimate.getZ());

      //create qf from doing where the gravity vector points and where the board thinks it is rn
      double rnAngle = correction*(mag(rnx, rny, rnz))*(PI/2);
      double qrnx = crossProduct2d(y, z, qgEstimate.getY(), qgEstimate.getZ());
      double qrny = -1*crossProduct2d(x, z, qgEstimate.getX(), qgEstimate.getZ());
      double qrnz = crossProduct2d(x, y, qgEstimate.getX(), qgEstimate.getY());

      Quaternion qfn = Quaternion(cos(rnAngle/2), qrnx*sin(rnAngle/2), qrny*sin(rnAngle/2), qrnz*sin(rnAngle/2));

      //update master quaternion, q0, with "instantaneous" rotation data quaternion qfn
        q0 = (q0.qMultiply(qfn));
        //make sure ts isnt over 1 bc then it explodes :<
        q0.qNormalize();
    }
    
  }
  
  // --- Gyroscope ---
  if (IMU.gyroscopeAvailable()) {

    nowTime = micros();

    IMU.readGyroscope(rdx, rdy, rdz);
    
  // --- Quaternions?! ---
    

    //Rotation Quaternion
    double magQvector = mag(rdx, rdy, rdz);
    double dt = (double)(nowTime - lastRunTime)/1000000;

    if(magQvector > 1){
      //convert to quaternion b, c, and d values
      qx = (rdx/magQvector)*sin((magQvector*dt/2)*(PI/180));
      qy = (rdy/magQvector)*sin((magQvector*dt/2)*(PI/180));
      qz = (rdz/magQvector)*sin((magQvector*dt/2)*(PI/180));

      //Apply rotation to q0:
        //set up the rotation quaternion
        Quaternion qf(cos((magQvector*dt/2)*(PI/180)), qx, qy, qz);

        //update master quaternion, q0, with "instantaneous" rotation data quaternion qf
        q0 = (q0.qMultiply(qf));
        //make sure ts isnt over 1 bc then it explodes :<
        q0.qNormalize();

        lastRunTime = nowTime;
    }
    //(qo^-1 x down x q0) to get actual rotation quaternions
    qgEstimate = (q0.inverse().qMultiply(down)).qMultiply(q0);
    qsEstimate = (q0.inverse().qMultiply(straight)).qMultiply(q0);

    //convert to pitch roll and yaw
    pitch = -1*atan2(qgEstimate.getX(), qgEstimate.getZ())*(180/PI);
    roll = -1*atan2(qgEstimate.getY(), qgEstimate.getZ())*(180/PI);
    yaw = -1*atan2(qsEstimate.getY(), qsEstimate.getX())*(180/PI);

    Serial.print("Pitch:");
    Serial.print(pitch);
    Serial.print(",Roll:");
    Serial.print(roll);
    Serial.print(",Yaw:");
    Serial.println(yaw);
  }
}
