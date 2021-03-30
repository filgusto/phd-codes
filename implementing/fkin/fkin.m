clear all;
clc;
close all;

%% preamble

% inform where the phd-codes folder is located for finding related code
folder_phd_codes = '/home/filipe/gitSources/doc/phd-codes';

addpath(strcat(folder_phd_codes,'/implementing/modelling'));

% loading robot dualquaternion modules
load('model_dq.mat');

%% Performing static forward kinematics

% computing fkin
%dq_res = dq_world_base * dq_base_gen3base * dq_gen3base_j1 * dq_j1_j2 * dq_j2_j3 * dq_j3_j4 * dq_j4_j5 * dq_j5_j6 * dq_j6_j7 * dq_j7_tcp
%dq_res = dq_world_base * dq_base_gen3base
dq_res =  dq_j7_tcp * dq_j6_j7 * dq_j5_j6 * dq_j4_j5 * dq_j3_j4 * dq_j2_j3 * dq_j1_j2 * dq_gen3base_j1 * dq_base_gen3base * dq_world_base


% extracting the th
ori = dq_res.q_p
tr = dq_res.extractTranslation
