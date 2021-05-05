clc 
clear all;
close all;

%% frames

% frame A
f_A = DualQuaternion();


% frame B
f_B = DualQuaternion();

% frame C
f_C = DualQuaternion();

%% pose h_c

% pose h_c
h_C_a = DualQuaternion();
h_C_a = h_C_a.setDQpureTranslation([0.7 1.5 0.5]);

h_C_b = DualQuaternion();
h_C_b = h_C_b.setDQpureRotation(deg2rad(30), [0 0 1]);

h_C_c = DualQuaternion();
h_C_c = h_C_c.setDQpureRotation(deg2rad(15), [1 0 0]);

h_C = DualQuaternion();
h_C = h_C_c * h_C_b * h_C_a;

%% transforms

% first transform
t_B_A = DualQuaternion();
t_B_A = t_B_A.setDQFromQuatAndTransl(quaternion(1, 0, 0, 0), [2 0 0]); % 90 degree positive rotation around z and a [1 1 1] translation


% second transform transform
t_C_B = DualQuaternion();
t_C_B = t_C_B.setDQFromQuatAndTransl(quaternion(1, 0, 0, 0), [0 0 1.5]); % 90 degree positive rotation around z and a [1 1 1] translation


%% setting frames poses

% frame B
f_B = t_B_A * f_A;

% frame C
f_C = t_C_B * f_B;

%% translating h_c

% right multiplication
h_A = h_C * t_C_B * t_B_A;

%% Plotting transformations

h_A.dq2th
h_C.dq2th

% right multiplication
figure
hold on;
plot([f_A, f_B, f_C]);
plot(h_A, 0.3);
% plot(h_C);
% plot(h_A);










