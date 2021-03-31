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

% computing fkin by left multiplication
% This way works as a glove
dq_res =  dq_j7_tcp * dq_mani_arr{7} * dq_mani_arr{6} * dq_mani_arr{5} * dq_mani_arr{4} * dq_mani_arr{3} * dq_mani_arr{2} * dq_mani_arr{1} * dq_base_gen3base * dq_world_base;

% extracting the th
ori_1 = dq_res.q_p
tr_1 = dq_res.extractTranslation


% computing fkin by right multiplication
% This way works with minor numeric error and the orientation should
% be conjugated
dq_res2 = dq_world_base' * dq_base_gen3base' * dq_mani_arr{1}' * dq_mani_arr{2}' * dq_mani_arr{3}' * dq_mani_arr{4}' * dq_mani_arr{5}' * dq_mani_arr{6}' * dq_mani_arr{7}' * dq_j7_tcp';

% extracting the th
ori_2 = dq_res2.q_p.conj;
tr_2 = dq_res2.extractTranslation;

