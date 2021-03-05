#include "Quaternion.h"

#define DEBUG

/* == Constructors == */
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


/* == Getters and Setters  == */

double Quaternion::getW() const { return w; }
void Quaternion::setW(double w) { Quaternion::w = w; }

double Quaternion::getX() const { return x; }
void Quaternion::setX(double x) { Quaternion::x = x; }

double Quaternion::getY() const { return y; }
void Quaternion::setY(double y) { Quaternion::y = y; }

double Quaternion::getZ() const { return z; }
void Quaternion::setZ(double z) { Quaternion::z = z; }


/* == Operators == */

// Sum operator
Quaternion Quaternion::operator+ (const Quaternion& rhs){
    return Quaternion(this->w + rhs.w,
                      this->x + rhs.x,
                      this->y + rhs.y,
                      this->z + rhs.z);
}


/* Multiplication operator */
Quaternion Quaternion::operator* (const Quaternion& rhs){
    Quaternion q_res;
    q_res.w = (this->w * rhs.w) - (this->x * rhs.x) - (this->y * rhs.y) - (this->z * rhs.z);
    q_res.x = (this->x * rhs.w) + (this->w * rhs.x) + (this->y * rhs.z) - (this->z * rhs.y);
    q_res.y = (this->w * rhs.y) - (this->x * rhs.z) + (this->y * rhs.w) + (this->z * rhs.x);
    q_res.z = (this->w * rhs.z) + (this->x * rhs.y) - (this->y * rhs.x) + (this->z * rhs.w);
    return q_res;
}


/* Multiplication by a double */
Quaternion Quaternion::operator* (const double rhs){
    return Quaternion(this->w * rhs,
                          this->x * rhs,
                          this->y * rhs,
                          this->z * rhs);
}


Quaternion Quaternion::operator*= (const double rhs){
    return Quaternion(this->w * rhs,
                      this->x * rhs,
                      this->y * rhs,
                      this->z * rhs);
}


/* == Manipulation == */

/* conjugate */
Quaternion Quaternion::Conjugate(){
    return Quaternion(this->w, -this->x, -this->y, -this->z);
}


/* Dot product */
// It might be a good idea to review this mathematics if something starts to go wrong
double Quaternion::Dot(const Quaternion rhs) {
    return ((this->w * rhs.w) + (this->x * rhs.x) + (this->y * rhs.y) + (this->z * rhs.z));
}


/* magnitude */
double Quaternion::Magnitude(){
    return sqrt(pow(this->w,2) + pow(this->x,2) + pow(this->y,2) + pow(this->z,2));
}


/* normalize */
Quaternion Quaternion::Normalize(){
    return *this * (1/this->Magnitude());
}


/* == Conversions == */

/* yaw pitch roll 2 quaternion */
// C'est une bonne idee de regarder si ca c'est bien
Quaternion Quaternion::YawPitchRoll2Quaternion(double yaw, double pitch, double roll){
    double w,x,y,z;
    double yaw_2 = yaw * 0.5f;
    double pitch_2 = pitch * 0.5f;
    double roll_2 = roll * 0.5f;

    // conversions
    w = (cos(roll_2) * cos(pitch_2) * cos(yaw_2)) + (sin(roll_2) * sin(pitch_2) * sin(yaw_2));
    x = (cos(roll_2) * sin(pitch_2) * cos(yaw_2)) + (sin(roll_2) * cos(pitch_2) * sin(yaw_2));
    y = (cos(roll_2) * cos(pitch_2) * sin(yaw_2)) - (sin(roll_2) * sin(pitch_2) * cos(yaw_2));
    z = (sin(roll_2) * cos(pitch_2) * cos(yaw_2)) - (cos(roll_2) * sin(pitch_2) * sin(yaw_2));

    return Quaternion(w,x,y,z);
}


/* quaternion to yaw pitch roll */
void Quaternion::Quaternion2YawPitchRoll(Quaternion q, double (&ret_ypr)[3]){

    // yaw
    double t3 = 2.0f * ((q.w * q.z) + (q.x * q.y));
    double t4 = 1.0f - 2.0f * ((q.y * q.y) + (q.z * q.z));
    ret_ypr[0] = atan2(t3, t4);

    // pitch
    double t2 = 2.0f * ((q.w * q.y) - (q.z * q.x));
    t2 = ((t2 > 1)  ?   1 : t2);
    t2 = ((t2 < -1) ?  -1 : t2);
    ret_ypr[1] = asin(t2);

    // roll
    double t0 = 2.0f * ((q.w * q.x) + (q.y * q.z));
    double t1 = 1.0f - 2.0f * ((q.x * q.x) + (q.y * q.y));
    ret_ypr[2] = atan2(t0, t1);
}
