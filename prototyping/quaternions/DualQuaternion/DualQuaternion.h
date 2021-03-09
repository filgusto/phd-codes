#ifndef DUALQUATERNION_DUALQUATERNION_H
#define DUALQUATERNION_DUALQUATERNION_H

#include <Quaternion.h>

# include <assert.h>

/* Class definition */
class DualQuaternion{
private:
    Quaternion q_scalar;
    Quaternion q_dual;

public:

    /* == Constructors == */
    DualQuaternion();
    DualQuaternion(Quaternion s, Quaternion d);
    DualQuaternion(Quaternion s, double (&trans)[3]);

    /* == Getters and Setters == */
    Quaternion getQscalar() const;
    void setQscalar(Quaternion q_scalar);
    Quaternion getQdual() const;
    void setQdual(Quaternion q_dual);

    /* == Operators == */
    DualQuaternion operator+ (const DualQuaternion& rhs);
    DualQuaternion operator* (const double rhs);
    DualQuaternion operator* (const DualQuaternion rhs);

    /*== Math methods == */
    float Dot (const DualQuaternion& rhs);
    DualQuaternion Normalize();

    /* == Conversions and utilities == */
    static void DualQuaternion2THMatrix(const DualQuaternion dq, double (&ret_M)[4][4]);
    static void ExtractRotM(DualQuaternion dq, double (&ret_M)[3][3]);
    static void ExtractTransl(DualQuaternion dq, double (&ret_T)[3]);
};

#endif //DUALQUATERNION_DUALQUATERNION_H
