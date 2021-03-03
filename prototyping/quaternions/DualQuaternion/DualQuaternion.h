#ifndef DUALQUATERNION_DUALQUATERNION_H
#define DUALQUATERNION_DUALQUATERNION_H

#include <Quaternion.h>

/* Class definition */
class DualQuaternion{
private:
    Quaternion q_scalar;
    Quaternion q_dual;

public:

    /* Constructors */
    DualQuaternion();
    DualQuaternion(Quaternion s, Quaternion d);

    /* Getters and Setters */
    Quaternion getQscalar() const;
    void setQscalar(Quaternion q_scalar);
    Quaternion getQdual() const;
    void setQdual(Quaternion q_dual);



};

#endif //DUALQUATERNION_DUALQUATERNION_H
