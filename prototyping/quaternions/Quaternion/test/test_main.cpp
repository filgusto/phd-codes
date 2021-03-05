//
// Created by Filipe Rocha on 01/03/21.
//
#include "../Quaternion.h"     // self header

// essential includes
#include <iostream>

// namespace
using namespace std;

// defining macros
#define PRINT_Q(q,n) cout << "q" << n << ": [w:" << q.getW() << ", x:" << q.getX() << ", y:"<< q.getY() << ", z:" << q.getZ() << "]\n";

int main(){

    cout << "Teste running..." << endl;

    // Creating two quaternions value arrays
    double q1_val[4] = {0.3, 2.1, -1.3, 33};
    double q2_val[4] = {11.2, 1, 7, -13};

    // Creating two quaternions objects
    Quaternion q1(q1_val);    // way 1
    Quaternion q2(q2_val[0], q2_val[1], q2_val[2], q2_val[3]);   // way 2

    PRINT_Q(q1,1);
    PRINT_Q(q2, 2);

    // q1 + q2
    Quaternion q3;
    q3 = q1 + q2;
    PRINT_Q(q3, 3);

    // q1 * q2
    Quaternion q4;
    q4 = q1 * q2;
    PRINT_Q(q4, 4);

    // conjugate
    Quaternion q5;
    q5 = q4.Conjugate();
    PRINT_Q(q5, 5);

    // magnitude
    double q4_mag;
    q4_mag = q4.Magnitude();
    cout << "q4_mag = " << q4_mag << endl;

    // multiplication by scalar
    Quaternion q6;
    q6 = q4 * 0.5;
    PRINT_Q(q6, 6);

    // Normalization
    Quaternion q7;
    q7 = q4.Normalize();
    PRINT_Q(q7, 7);
    cout << "q7_mag = " << q7.Magnitude() << endl;

    // Dot product
    double dot_prod;
    dot_prod = q7.Dot(q5);
    cout << "q7 dot q5 = " << dot_prod << endl;

    // convert from yaw pitch roll to quaternion
    cout << "\n" ;
    Quaternion q8;
    q8 = Quaternion::YawPitchRoll2Quaternion(0, 3.14f, 0);
    PRINT_Q(q8, 8);

    // convert from quaternion to yaw pitch roll
    double q8_ypr[3];
    Quaternion::Quaternion2YawPitchRoll(q8, q8_ypr);
    cout << "Quaternion q8 to yaw pitch roll: " << q8_ypr[0] << ", " << q8_ypr[1] << ", " << q8_ypr[2] << endl;


    return 0;
}
