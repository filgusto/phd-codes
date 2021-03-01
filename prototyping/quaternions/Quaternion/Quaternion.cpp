#include "Quaternion.h"
#include <iostream>
#include <cmath>

#define DEBUG

/* Constructors */
Quaternion::Quaternion(){
    this->w = 0;
    this->x = 0;
    this->y = 0;
    this->z = 0;
}


Quaternion::Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}


Quaternion::Quaternion(double *q){
    this->w = *(q);
    this->x = *(q+1);
    this->y = *(q+2);
    this->z = *(q+3);
}


/* Getters and setters */
double Quaternion::getW() const { return w; }
void Quaternion::setW(double w) { Quaternion::w = w; }

double Quaternion::getX() const { return x; }
void Quaternion::setX(double x) { Quaternion::x = x; }

double Quaternion::getY() const { return y; }
void Quaternion::setY(double y) { Quaternion::y = y; }

double Quaternion::getZ() const { return z; }
void Quaternion::setZ(double z) { Quaternion::z = z; }

/* Operators */

// Sum operator
Quaternion Quaternion::operator+ (const Quaternion& rhs){
    Quaternion q_res;
    q_res.w = this->w + rhs.w;
    q_res.x = this->x + rhs.x;
    q_res.y = this->y + rhs.y;
    q_res.z = this->z + rhs.z;
    return q_res;
}


// Multiplication operator
Quaternion Quaternion::operator* (const Quaternion& rhs){
    Quaternion q_res;
    q_res.w = (this->w * rhs.w) - (this->x * rhs.x) - (this->y * rhs.y) - (this->z * rhs.z);
    q_res.x = (this->x * rhs.w) + (this->w * rhs.x) + (this->y * rhs.z) - (this->z * rhs.y);
    q_res.y = (this->w * rhs.y) - (this->x * rhs.z) + (this->y * rhs.w) + (this->z * rhs.x);
    q_res.z = (this->w * rhs.z) + (this->x * rhs.y) - (this->y * rhs.x) + (this->z * rhs.w);
    return q_res;
}


Quaternion Quaternion::operator* (const double rhs){
    return Quaternion(this->w * rhs,
                          this->x * rhs,
                          this->y * rhs,
                          this->z * rhs);
}


Quaternion Quaternion::Conjugate(){
    return Quaternion(this->w, -this->x, -this->y, -this->z);
}


double Quaternion::Magnitude(){
    return sqrt(pow(this->w,2) + pow(this->x,2) + pow(this->y,2) + pow(this->z,2));
}

Quaternion Quaternion::Normalize(){
    return *this * (1/this->Magnitude());
}