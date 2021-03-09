//
// Created by filipe on 26/02/2021.
//
#include "../DualQuaternion.h"

#include <iostream>
#include <Quaternion.h>

using namespace std;

/* Defining functions */

/* for printing dual-quaternions */
void print_dq(DualQuaternion& dq, int n){
    Quaternion q_s = dq.getQscalar();
    Quaternion q_d = dq.getQdual();
    cout << "dq" << n << ": [w_s:" << q_s.getW() << ", x_s:" << q_s.getX() << ", y_s:"<< q_s.getY() << ", z_s:" << q_s.getZ() <<
         "] + [w_d:" << q_d.getW() << ", x_d:" << q_d.getX() << ", q_d:"<< q_d.getY() << ", z_d:" << q_d.getZ() << "]dual \n" ;
}

/* for printing homogeneous transform matrix */
void print_TH(double (M)[4][4], int n){
    cout << "\n Homogeneous Transform" << n <<
    "\n" << M[0][0] << " " <<  M[0][1] << " " <<  M[0][2] << " " <<  M[0][3] << " " <<
    "\n" << M[1][0] << " " <<  M[1][1] << " " <<  M[1][2] << " " <<  M[1][3] << " " <<
    "\n" << M[2][0] << " " <<  M[2][1] << " " <<  M[2][2] << " " <<  M[2][3] << " " <<
    "\n" << M[3][0] << " " <<  M[3][1] << " " <<  M[3][2] << " " <<  M[3][3] << "\n" ;
}

/* for printing quaternions */
#define PRINT_Q(q,n) cout << "q" << n << ": [w:" << q.getW() << ", x:" << q.getX() << ", y:"<< q.getY() << ", z:" << q.getZ() << "]\n";


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


    // Creating dual-quaternion from rotation and translation vector
    Quaternion q3_r(1,0,0,0);
    double q3_t[3] = {1.1, 1.2, 1.3};
    DualQuaternion dq3(q3_r, q3_t);
    print_dq(dq3, 3);


    // Adding dual-quaternions
    DualQuaternion dq4;
    dq4 = dq2 + dq3;
    print_dq(dq4, 4);


    // Dot product
    cout << "Dot product dq2 dot dq4 = " << dq2.Dot(dq4) << endl;


    // Multiplying by an scalar
    DualQuaternion dq5;
    double dq4_s = 4.0;
    dq5 = dq4 * dq4_s;
    print_dq(dq5, 5);


    // Normalizing
    DualQuaternion dq6;
    dq6 = dq5.Normalize();
    print_dq(dq6, 6);


    // Multiplying dual-quaternions
    DualQuaternion dq7;
    dq7 = dq4 * dq5;
    print_dq(dq7, 7);

    /* Performing final test */
    cout << "\n\n Final test \n";

    // first transformation
    Quaternion dq_0_q = Quaternion::YawPitchRoll2Quaternion(1.0, 2.0, 3.0);
    double dq_0_v[3] = {10.0f, 30.0f, 90.0f};
    DualQuaternion dq_0(dq_0_q, dq_0_v);
    print_dq(dq_0, 0);

    // second transformation
    Quaternion dq_1_q = Quaternion::YawPitchRoll2Quaternion(-1.0f, 3.0f, 2.0f);
    double dq_1_v[3] = {30, 40, 190};
    DualQuaternion dq_1(dq_1_q, dq_1_v);
    print_dq(dq_1, 1);

    // third transformation
    Quaternion dq_2_q = Quaternion::YawPitchRoll2Quaternion(2.0f, 3.0f, 1.5f);
    double dq_2_v[3] = {5.0f, 20.0f, 66.0f };
    DualQuaternion dq_2(dq_2_q, dq_2_v);
    print_dq(dq_2, 2);

    // performing transformation
    DualQuaternion dq_3;
    dq_3 = dq_0 * dq_1 * dq_2;
    print_dq(dq_3, 3);

    // converting the result to homogeneous transform format
    double mat[4][4];
    DualQuaternion::DualQuaternion2THMatrix(dq_3, mat);
    print_TH(mat, 1);

    cout << "\n\nMain finished..." << endl;
    
    return 0;
}