% Right multiplication left multiplication
clc 
clear all;
close all;


% initial condition
c = DualQuaternion();

% first transform
t1 = DualQuaternion();
t1 = t1.setDQFromQuatAndTransl(quaternion(7.071068e-01, 0, 0, 7.071068e-01), [1 1 1]); % 90 degree positive rotation around z and a [1 1 1] translation

% second transform
t2 = DualQuaternion();
t2 = t2.setDQFromQuatAndTransl(quaternion(7.071068e-01, 7.071068e-01, 0, 0), [1 1 1]); % 90 degree positive rotation around x and a [1 1 1] translation


%% transforming

% right multiplication
res1_1 = c * t1;
res1_2 = res1_1 * t2;

% left multiplication
res2_1 = t1 * c;
res2_2 = t2 * res2_1;


%% Plotting both transformations

res1_2.dq2th
res2_2.dq2th

% right multiplication
figure
hold on;
plot(c)
plot([res1_1, res1_2], 0.3);
title('Right multiplication');
disp('Right multiplications rotates w.r.t. original frame');

% left multiplication
figure
hold on;
plot(c)
plot([res2_1, res2_2], 0.3);
title('Sequence of pose transforms - DQ left multiplication');
disp('Left multiplications rotates w.r.t. the last computed frame');










