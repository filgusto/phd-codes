%% preamble

% for using directly Dual Quaternion library namespace
include_namespace_dq;

% inform where the phd-codes folder is located for finding related code
folder_phd_codes = '/home/filipe/gitSources/doc/phd-codes';

% adding subfolder to path
addpath('./lib/');
addpath(strcat(folder_phd_codes,'/implementing/lib/ros')); 
addpath(strcat(folder_phd_codes,'/implementing/lib/plot'));
addpath(strcat(folder_phd_codes,'/implementing/lib/dq'));
addpath(strcat(folder_phd_codes,'/prototyping/dq-robotics/lib'));