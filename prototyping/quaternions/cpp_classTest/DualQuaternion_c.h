//
// Created by Filipe Rocha on 18/02/21.
//

#ifndef QUATERNION_CLASS_DUALQUATERNION_C_H
#define QUATERNION_CLASS_DUALQUATERNION_C_H

#include <boost/qvm/quat.hpp>
#include <boost/qvm/vec.hpp>
#include <boost/qvm/mat.hpp>

class DualQuaternion_c {
    //boost::qvm::quat<> m_real;
    //boost::qvm::quat m_dual;

public:
    DualQuaternion_c();
    DualQuaternion_c(Quaternion r, Quaternion d);
    DualQuaternion_c(Quaternion r, Vector3 t);

    static DualQuaternion_c operator* (DualQuaternion_c q, float scale);
    static DualQuaternion_c operator + (DualQuaternion_c lhs, DualQuaternion_c rhs);
    static DualQuaternion_c operator * (DualQuaternion_c lhs, DualQuaternion_c rhs);
    static float Dot(DualQuaternion_c a, DualQuaternion_c b);
    static DualQuaternion_c Normalize(DualQuaternion_c q);
    static DualQuaternion_c Conjugate(DualQuaternion_c q);
    static Quaternion GetRotation(DualQuaternion_c q);
    static Vector3 GetTranslation(DualQuaternion_c q);
    static Matrix DualQuaternionToMatrix(DualQuaternion_c q);
};

#endif //QUATERNION_CLASS_DUALQUATERNION_C_H
