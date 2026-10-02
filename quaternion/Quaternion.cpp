#include "Arduino.h"
#include "Quaternion.h"

Quaternion::Quaternion(double ww, double xx, double yy, double zz){
      w = ww;
      x = xx;
      y = yy;
      z = zz;
}

  

    void Quaternion::qPrint(){
      
      Serial.print("Quaternion: ");
      Serial.print(w);
      Serial.print(" + ");
      Serial.print(x);
      Serial.print("i + ");
      Serial.print(y);
      Serial.print("j + ");
      Serial.print(z);
      Serial.println("k");
    }

    Quaternion Quaternion::inverse() {
      return Quaternion(w, -x, -y, -z);
    }

    Quaternion Quaternion::qMultiply(Quaternion other){
      double a0 = this->w;
      double b0 = this->x;
      double c0 = this->y;
      double d0 = this->z;

      double aother = other.w;
      double bother = other.x;
      double cother = other.y;
      double dother = other.z;

      double af = a0*aother - b0*bother - c0*cother - d0*dother;
      double bf = a0*bother + b0*aother + c0*dother - d0*cother;
      double cf = a0*cother - b0*dother + c0*aother + d0*bother;
      double df = a0*dother + b0*cother - c0*bother + d0*aother;
      
      Quaternion qNew(af, bf, cf, df);
      //Serial.print("Multiplied ");
      //qNew.qPrint();
      return qNew;
      
    }

    void Quaternion::qNormalize(){
      double qMag = sqrt(w*w + x*x + y*y + z*z);
      w = w/qMag;
      x = x/qMag;
      y = y/qMag;
      z = z/qMag;
    }

    double Quaternion::getW(){
      return(w);
    }

    double Quaternion::getX(){
     return(x); 
    }

    double Quaternion::getY(){
     return(y); 
    }
    
    double Quaternion::getZ(){
     return(z); 
    }

    void Quaternion::plotVector(){
      Serial.print("X:");
      Serial.print(x);
      Serial.print(",");
      Serial.print("Y:");
      Serial.print(y);
      Serial.print(",");
      Serial.print("Z:");
      Serial.println(z); 
    }
