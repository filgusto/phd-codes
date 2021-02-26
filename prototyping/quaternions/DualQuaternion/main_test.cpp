//
// Created by filipe on 26/02/2021.
//
#include <iostream>
#include "dualquaternion.h"
using namespace std;

int main(){

    cout << "Main running..." << endl;

    DualQuaternion q1;
    q1.setA(3.3);
    cout << " A value: " << q1.getA() << endl;

    return 0;
}