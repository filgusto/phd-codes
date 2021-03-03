#include "DualQuaternion.h"

/* Essential includes */
#include <iostream>


/* Constructors */
DualQuaternion::DualQuaternion() {
    this->q_scalar = Quaternion(0,0,0,1);
    this->q_dual = Quaternion(0,0,0,0);
}

DualQuaternion::DualQuaternion(Quaternion s, Quaternion d){
    this->q_scalar = s.Normalize();
    this->q_dual = d;
}


/* Getters and Setters */
Quaternion DualQuaternion::getQscalar() const {return this->q_scalar; };
void DualQuaternion::setQscalar(Quaternion q_scalar) {this->q_scalar = q_scalar; };
Quaternion DualQuaternion::getQdual() const {return this->q_dual; };
void DualQuaternion::setQdual(Quaternion q_dual) {this->q_dual = q_dual; };

/* Operators */
