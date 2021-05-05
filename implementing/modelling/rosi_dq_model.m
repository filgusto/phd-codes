clear all;
clc;
close all;

%% preamble

% inform where the phd-codes folder is located for finding related code
folder_phd_codes = '/home/filipe/gitSources/doc/phd-codes';

addpath(strcat(folder_phd_codes,'/implementing/lib/dq'));
addpath('./lib');

%% DEFINING FRAMES

%% World frame
dq_world = DualQuaternion();


%% ROSI frames

% -- rosi base w.r.t. world frame
qo_mundo_base = quaternion([-2.9802304624127e-08, 0, 0, 1]);
t_mundo_base = [0, 0, 0.24099996685982];

dq_world_base = dq_relative_pose(qo_mundo_base, t_mundo_base);


% -- manipulator base w.r.t. robot base frame
qo_base_gen3base = quaternion([1, 0, 0, 0]);
t_base_gen3base = [-0.13597910106182, -0.067999750375748, 0.26568099856377];

dq_base_arm = dq_relative_pose(qo_base_gen3base, t_base_gen3base);


%% Manipulator frames

% -- manipulator joint 1 w.r.t. manipulator base
qo_gen3base_j1 = quaternion([0.006186225451529, 0.0036798391956836, -0.99997401237488, -0.00048272835556418]);
t_gen3base_j1 = [-0.00079936918336898, 3.560958430171e-05, 0.071031183004379];

dq_gen3base_j1 = dq_relative_pose(qo_gen3base_j1, t_gen3base_j1);



% -- manipulator joint 2 w.r.t. joint 1
qo_j1_j2 = quaternion([0.6743785738945, 0.67438089847565, 0.21263155341148, -0.21263082325459]);
t_j1_j2 = [0, 0.0053750053048134, -0.12838006019592];

dq_j1_j2 = dq_relative_pose(qo_j1_j2, t_j1_j2);



% -- manipulator joint 3 w.r.t. joint 2
qo_j2_j3 = quaternion([0.70710557699203, -0.70710808038712, -5.9604644775391e-08, -1.4901161193848e-08]);
t_j2_j3 = [5.9604644775391e-08, -0.21037989854813, -0.0063750222325325];

dq_j2_j3 = dq_relative_pose(qo_j2_j3, t_j2_j3);



% -- manipulator joint 4 w.r.t. joint 3
qo_j3_j4 = quaternion([0.37992748618126, 0.37992885708809, 0.59636896848679, -0.59636670351028]);
t_j3_j4 = [1.1920928955078e-07, 0.0063749849796295, -0.21038019657135];

dq_j3_j4 = dq_relative_pose(qo_j3_j4, t_j3_j4);



% -- manipulator joint 5 w.r.t. joint 4
qo_j4_j5 = quaternion([0.70710557699203, -0.70710808038712, -1.3411045074463e-07, -2.9802322387695e-08]);
t_j4_j5 = [0, -0.2084299325943, -0.0063750147819519];

dq_j4_j5 = dq_relative_pose(qo_j4_j5, t_j4_j5);



% -- manipulator joint 6 w.r.t. joint 5
qo_j5_j6 = quaternion([0.60926270484924, 0.60926508903503, -0.35888448357582, 0.3588829934597]);
t_j5_j6 = [5.9604644775391e-08, 0.00017501413822174, -0.10593014955521];

dq_j5_j6 = dq_relative_pose(qo_j5_j6, t_j5_j6);



% -- manipulator joint 7 w.r.t. joint 6
qo_j6_j7 = quaternion([-0.49999907612801, 0.50000089406967, -0.50000089406967, -0.49999910593033]);
t_j6_j7 = [0, -0.10593006014824, -0.00017508864402771];

dq_j6_j7 = dq_relative_pose(qo_j6_j7, t_j6_j7);



% -- manipulator tcp w.r.t. joint 7
qo_j7_tcp = quaternion([1.1920926112907e-07, -1, 5.4538236327062e-06, -1.6987319213513e-06]);
t_j7_tcp = [0.041204608976841, 0.060173213481903, -0.25342667102814];

dq_j7_tcp = dq_relative_pose(qo_j7_tcp, t_j7_tcp);


%% Mounting manipulator dual quaternion array
dq_arm_arr = {dq_gen3base_j1, dq_j1_j2, dq_j2_j3, dq_j3_j4, dq_j4_j5, dq_j5_j6, dq_j6_j7, dq_j7_tcp};


%% Plotting the robot

% plot([dq_world, dq_world_base, dq_base_arm, dq_gen3base_j1, dq_j1_j2, dq_j2_j3, dq_j3_j4, dq_j4_j5, dq_j5_j6, dq_j6_j7, dq_j7_tcp]);

%% Saving variables

save('model_dq', 'dq_world', 'dq_world_base', 'dq_base_arm','dq_arm_arr');

clear;
load('model_dq.mat');

