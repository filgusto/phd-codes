//
// Created by filipe on 26/02/2021.
//
#include "../DualQuaternion.h"

#include <iostream>
#include <Quaternion.h>

using namespace std;

/* Defining functions */
void print_dq(DualQuaternion& dq, int n){
    Quaternion q_s = dq.getQscalar();
    Quaternion q_d = dq.getQdual();
    cout << "dq" << n << ": [w_s:" << q_s.getW() << ", x_s:" << q_s.getX() << ", y_s:"<< q_s.getY() << ", z_s:" << q_s.getZ() <<
         "] [w_d:" << q_d.getW() << ", x_d:" << q_d.getX() << ", q_d:"<< q_d.getY() << ", z_d:" << q_d.getZ() << "]\n" ;
}

#define PRINT_DQ(dq,n) cout << "dq" << n << ": [ws:" << dq.getQscalar() << ", x:" << q.getX() << ", y:"<< q.getY() << ", z:" << q.getZ() << "]\n";

int main(){

    cout << "Test running..." << endl;


    // Creating dual-quaternion from quaternions
    Quaternion q1_s(0,0,1,1);
    Quaternion q1_d(0,0,0,0);

    DualQuaternion dq1(q1_s, q1_d);

    print_dq(dq1, 1);


    // Creating unitary dual-quaternion
    DualQuaternion dq2;

    print_dq(dq2, 2);




    cout << "Main running..." << endl;
    
    return 0;
}