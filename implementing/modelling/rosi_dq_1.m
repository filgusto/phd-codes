% Code based on the snippet contained at the article
% Adorno, 2020, DQ Robotics: a Library for Robot Modeling and Control

clear all;
clc;

% for using directly Dual Quaternion library namespace
include_namespace_dq;

%% Robot parameters

% wheel radius
wheel_radius = 0.1324;

% distance between wheels
wheels_side_distance = 0.5749;

%% Defining the arm

% gen3 DH parameters
arm_DH_theta = [0, pi, pi, pi, pi, pi, pi];
arm_DH_d = [-0.2848, -0.0118, -0.4208, -0.0128, -0.3143, 0.0, -0.1674];
arm_DH_a = [0, 0, 0, 0, 0, 0, 0];
arm_DH_alpha = [pi/2, pi/2, pi/2, pi/2, pi/2, pi/2, pi];

% arm dh matrix
arm_DH_matrix = [arm_DH_theta; arm_DH_d; arm_DH_a; arm_DH_alpha];

% criando o braco cinematicamente
arm = DQ_SerialManipulator(arm_DH_matrix, 'standard');


%% Defining the mobile base

% creating as a differential drive robot 
base = DQ_DifferentialDriveRobot(wheel_radius, wheels_side_distance);


%% Coupling both together

% arm displacement 
x_bm = 1 + E_ * 0.5 * (-0.136*i_ -0.068*j_ -0.2756*k_);
base.set_frame_displacement(x_bm);

% coupling
robot = DQ_WholeBody(base);
robot.add(arm);




















