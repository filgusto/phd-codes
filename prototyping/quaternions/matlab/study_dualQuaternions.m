% OK
% The dual-quaternion multiplication is running properly!
% 
% Concerns
% I'm still trying to compare matricial and dual-quaternions transform...
% still no success

clear all;
clc;
close all;
addpath('./lib');

%% Quaternion Transformation
dq0 = [0.43595284, 0.31062245, 	-0.71828705, 0.44443506, -10.778385,  41.16921, 18.295128, 11.3671055].';
dq1 = [-0.36887133, 0.44443512, -0.7549338, 0.31062245, -21.076984, 72.39809, 30.184574, -55.255486].';
dq2 = [0.6001072, 0.43491626, -0.32381588, -0.58810073, 21.558191, 6.3051853, 21.82356, 14.6448345].';

% performing dual-quaternion transformation
dq_out = dquat_cross(dq0, dquat_cross(dq1,dq2));


%% Equivalent computation in matricial form

% converting transform quaternions to homogeneous transforms
th0 = dquat_to_th(dq0);
th1 = dquat_to_th(dq1);
th2 = dquat_to_th(dq2);

% performing matricial transformations
th_out = th0 * th1 * th2;

%% Comparison

% converting resulting dual- quaternion to an homogeneous transform
dq_out_th = dquat_to_th(dq_out);

dq_out_th
th_out


% input point homogeneous transform representation
%T_point_in = th_from_pose(p_in_pos, p_in_ori);

% mounting homogeneous transform for the desired transformation
%T_rot = th_from_pose_transform([0;0;0], t_rot_axis, t_rot_angle);
%T_transl = th_from_pose_transform(t_transl, t_rot_axis, 0);

% performing matrix transformation
%res_T = T_transl * T_rot * T_point_in;


% Input spacial pose
% p_in_pos = [1; 1; 1];
% p_in_ori = [1; 0; 0; 0];
% 
% % Desired transformation
% t_transl = [1; 1; 1];
% t_rot_axis = [1; 0; 0];
% t_rot_angle = deg2rad(90);
% 
% 



