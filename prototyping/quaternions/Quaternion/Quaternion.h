#ifndef QUATERNION_QUATERNION_H
#define QUATERNION_QUATERNION_H

#include <cmath>

/* Class definition */
class Quaternion{
private:

    /* quaternion members */
    double w;
    double x;
    double y;
    double z;

public:

    /* Constructors */
    Quaternion();
    Quaternion(double w, double x, double y, double z);
    Quaternion(double *q);

    /* Getters and Setters */
    double getW() const;
    void setW(double w);
    double getX() const;
    void setX(double x);
    double getY() const;
    void setY(double y);
    double getZ() const;
    void setZ(double z);

    /* Operators */
    Quaternion operator+ (const Quaternion& rhs);
    Quaternion operator* (const Quaternion& rhs);
    Quaternion operator* (const double rhs);

    /* Manipulation */
    Quaternion Conjugate();
    double Magnitude();
    Quaternion Normalize();

};

#endif //QUATERNION_QUATERNION_H
