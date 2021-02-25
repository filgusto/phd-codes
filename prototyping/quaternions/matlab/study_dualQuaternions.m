% OK
% The dual-quaternion multiplication is running properly!
% Il reste ecrire le reste pour comparer avec les transformé homogeéne.

clear all;
clc;
close all;
addpath('./lib');

%% Quaternion Transformation
dq0 = [0.43595284, 0.31062245, 	-0.71828705, 0.44443506, -10.778385,  41.16921, 18.295128, 11.3671055].';
dq1 = [-0.36887133, 0.44443512, -0.7549338, 0.31062245, -21.076984, 72.39809, 30.184574, -55.255486].';
dq2 = [0.6001072, 0.43491626, -0.32381588, -0.58810073, 21.558191, 6.3051853, 21.82356, 14.6448345].';

% performing dual-quaternion transformation
dq_out = dquat_cross(dq0, dquat_cross(dq1,dq2))

% Input spacial pose
p_in_pos = [1; 1; 1];
p_in_ori = [1; 0; 0; 0];

% Desired transformation
t_transl = [1; 1; 1];
t_rot_axis = [1; 0; 0];
t_rot_angle = deg2rad(90);


%% Computation dual-quaternion operation

% input point dual-quaternion representation
dq_point_in = dquat_from_pose(p_in_pos, p_in_ori);

% obtaining transform quaternion
dq_trans = dquat_from_transform(t_transl, t_rot_axis, t_rot_angle);

% obtaining the transform conjugate
dq_trans_c = dquat_conjugate(dq_trans);

% performing dual-quaternion transformation
dq_point_out = dquat_cross(dq_trans, dquat_cross(dq_point_in, dq_trans_c));

%% Equivalent computation in matricial form

% input point homogeneous transform representation
T_point_in = th_from_pose(p_in_pos, p_in_ori);

% mounting homogeneous transform for the desired transformation
T_rot = th_from_pose_transform([0;0;0], t_rot_axis, t_rot_angle);
T_transl = th_from_pose_transform(t_transl, t_rot_axis, 0);

% performing matrix transformation
res_T = T_transl * T_rot * T_point_in;

%% Comparing results in common ground

% converting dual-quaternion output to matrix
res_dq = dquat_to_th(dq_point_out);




