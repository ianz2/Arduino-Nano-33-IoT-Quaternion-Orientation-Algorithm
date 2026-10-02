#ifndef Quaternion_h
#define Quaternion_h
#include "Arduino.h"

class Quaternion{
  public:
    Quaternion(double ww, double xx, double yy, double zz);
    void qPrint();
    Quaternion inverse();
    Quaternion qMultiply(Quaternion other);
    void qNormalize();
    double getW();
    double getX();
    double getY();
    double getZ();
    void plotVector();

  private:
    double w, x, y, z;

};
#endif