% Main for testing the DualQuaternion class
clear all;
clc;
close all;

% first transform
q1_p = quaternion(0.4359, 0.3106, -0.7182, 0.4444);
q1_v = [10, 30, 90];
dq1 = DualQuaternion();
dq1 = dq1.setDQFromQuatAndTransl(q1_p, q1_v);

% second transform
q2_p = quaternion(-0.3688, 0.4444, -0.7549, 0.3106);
q2_v = [30, 40, 190];
dq2 = DualQuaternion();
dq2 = dq2.setDQFromQuatAndTransl(q2_p, q2_v);

% third transform 
q3_p = quaternion(0.6, 0.4349, -0.3238, -0.5881);
q3_v = [5, 20, 66];
dq3 = DualQuaternion();
dq3 = dq2.setDQFromQuatAndTransl(q3_p, q3_v);

% performing transforms
dq_res1 = dq1 * dq2 * dq3;

% normalizing
dq_res1_normalized = dq_res1.normalize;

% converting to TH matrix
TH = dq_res1.dq2th