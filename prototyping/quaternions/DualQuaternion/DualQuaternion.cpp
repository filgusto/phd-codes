#include "DualQuaternion.h"

#include <iostream>


/* == Constructors == */

DualQuaternion::DualQuaternion() {
    this->q_scalar = Quaternion(0,0,0,1);
    this->q_dual = Quaternion(0,0,0,0);
}


/* Creates a dual-quaternion from two quaternions */
DualQuaternion::DualQuaternion(Quaternion s, Quaternion d){
    this->q_scalar = s.Normalize();
    this->q_dual = d;
}


/* Creates a dual-quaternion from an orientation and a translation 3D vector */
DualQuaternion::DualQuaternion(Quaternion s, double (&trans)[3]) {
    this->q_scalar = s.Normalize();
    this->q_dual = (Quaternion(0, trans[0], trans[1], trans[2]) * this->q_scalar) * 0.5f;
};


/* == Getters and Setters == */

Quaternion DualQuaternion::getQscalar() const {return this->q_scalar; };
void DualQuaternion::setQscalar(Quaternion q_scalar) {this->q_scalar = q_scalar; };
Quaternion DualQuaternion::getQdual() const {return this->q_dual; };
void DualQuaternion::setQdual(Quaternion q_dual) {this->q_dual = q_dual; }


/* == Operators == */

/* Sum */
DualQuaternion DualQuaternion::operator+ (const DualQuaternion& rhs){
    return  DualQuaternion(this->q_scalar + rhs.q_scalar, this->q_dual + rhs.q_dual);
}


/* Multiplication by a scalar */
DualQuaternion DualQuaternion::operator* (const double rhs){
    /*Quaternion s = this->q_scalar * rhs;
    Quaternion d = this->q_dual * rhs;
    return DualQuaternion(s, d); */

   return DualQuaternion(this->q_scalar * rhs,
                          this->q_dual * rhs);
}


/* Multiplication by another dual-quaternion */
DualQuaternion DualQuaternion::operator* (DualQuaternion rhs){
    return DualQuaternion(rhs.q_scalar * this->q_scalar,
                          (rhs.q_dual * this->q_scalar) + (rhs.q_scalar * this->q_dual));
}


/* == Math methods == */

// Dot product
float DualQuaternion::Dot (const DualQuaternion& rhs){
    return this->q_scalar.Dot(rhs.q_scalar);
}


// Normalize
DualQuaternion DualQuaternion::Normalize(){
    float mag = this->q_scalar.Dot(this->q_scalar);
    assert(mag > 0.000001f);
    std::cout << "CHECK mag " << mag << std::endl;
    return DualQuaternion(  this->q_scalar * (1.0f/mag),
                            this->q_dual   * (1.0f/mag));
}


/* == Conversions == */

/* Quaternion to Homogeneous Transform Matrix */
void DualQuaternion::DualQuaternion2THMatrix(DualQuaternion dq, double (&ret_M)[4][4]){

    // normalizes the input quaternion
    DualQuaternion dq_n = dq.Normalize();

    // extracts the rotation matrix and saves to ret_rotM
    double ret_rotM[3][3];
    DualQuaternion::ExtractRotM(dq, ret_rotM);

    // inserts rotM values inside the homogeneous transform
    for (int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            ret_M[i][j] = ret_rotM[i][j];
        }
    }

    // extracts the translation vector and saves to ret_transl
    double ret_transl[3];
    DualQuaternion::ExtractTransl(dq, ret_transl);

    for (int i=0; i<3; i++){
        ret_M[i][3] = ret_transl[i];
    }

    // finishing mounting the homogeneous transform matrix
    ret_M[3][0] = 0;
    ret_M[3][1] = 0;
    ret_M[3][2] = 0;
    ret_M[3][3] = 1;
}

/* Extracts the Rotation matrix from a quaternion */
void DualQuaternion::ExtractRotM(DualQuaternion dq, double (&ret_M)[3][3]){

    // normalizes the input quaternion
    DualQuaternion dq_n = dq.Normalize();

    // Extracting rotation quaternion
    Quaternion q_s = dq.getQscalar();
    double w = q_s.getW();
    double x = q_s.getX();
    double y = q_s.getY();
    double z = q_s.getZ();

    // Converting rotation information to rotation matrix
    ret_M[0][0] = w * w + x * x - y * y - z * z;
    ret_M[1][0] = 2 * x * y + 2 * w * z;
    ret_M[2][0] = 2 * x * z - 2 * w * y;
    ret_M[0][1] = 2 * x * y - 2 * w * z;
    ret_M[1][1] = w * w + y * y - x * x - z * z;
    ret_M[2][1] = 2 * y * z + 2 * w * x;
    ret_M[0][2] = 2 * x * z + 2 * w * y;
    ret_M[1][2] = 2 * y * z - 2 * w * x;
    ret_M[2][2] = w * w + z * z - x * x - y * y;

}

/* Extracts the translation vector */
void DualQuaternion::ExtractTransl(DualQuaternion dq, double (&ret_T)[3]){

    // adequates the translation vector
    Quaternion t = (dq.getQdual() * 2.0f) * (dq.getQscalar()).Conjugate();

    // Fills the returning vector
    ret_T[0] = t.getX();    // x coordinate
    ret_T[1] = t.getY();    // y coordinate
    ret_T[2] = t.getZ();    // z coordinate
}


