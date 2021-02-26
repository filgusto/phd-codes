#include "dualquaternion.h"

#include <iostream>

// Class constructor
DualQuaternion::DualQuaternion()
{
    a = 9;
}

int DualQuaternion::getA() const {
    return a;
}

void DualQuaternion::setA(int a) {
    DualQuaternion::a = a;
}
