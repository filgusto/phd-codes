clear all;
clc;
close all;

%% preamble

% inform where the phd-codes folder is located for finding related code
folder_phd_codes = '/home/filipe/gitSources/doc/phd-codes';

addpath(strcat(folder_phd_codes,'/implementing/modelling'));

% loading robot dualquaternion modules
load('model_dq2.mat');

%% Performing static forward kinematics

% computing fkin by left multiplication
% This way works as a glove
dq_res = dq_world_base * dq_base_arm * dq_arm_arr{1} * dq_arm_arr{2} * dq_arm_arr{3} * dq_arm_arr{4} * dq_arm_arr{5} * dq_arm_arr{6} * dq_arm_arr{7} * dq_arm_arr{8}


% actual values
qo = quaternion([-0.5033620595932, -0.50030332803726, 0.49714663624763, 0.49916788935661]);
t = [-0.62334549427032, -0.0035411268472672, 0.71529132127762];

dq_a = dq_relative_pose(qo, t)

% extracting the th
%ori_1 = dq_res.q_p
%tr_1 = dq_res.extractTranslation
