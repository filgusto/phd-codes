%% preamble

clear all;
close all;

% for using directly Dual Quaternion library namespace
include_namespace_dq;

addpath('./lib');

%% parameters

% declaring set-point
r = cos(pi /2) + j_*sin(pi /2);
p = 0.1* i_ + 0.2* j_ + 0.3* k_;
xd = r + E_ *0.5* p*r;

% setting current joints position
q = [0, 0, 0, 0, 0.3770 , 0.1257 , -0.5655 , 0, 0, 0].';
% q = [q_base, q_manipulador]
% q_base = [pos_x, pox_y, ang_z]
% q_manipulador = [q_1, q_2, q_3, q_4, q_5, q_6, q_7]

% sampling time
T = 0.001;

% controller gain
gain = 10;

% error tolerance
error_tol = 0.01;

% wheel radius
wheel_radius = 0.1324;

% distance between wheels
wheels_side_distance = 0.5749;


%% defining models

% loading kuka manipulator 7DoF Manpulator
lwr4 = KukaLwr4Robot.kinematics();

% creating as a differential drive robot 
base = DQ_DifferentialDriveRobot(wheel_radius, wheels_side_distance);


%% coupling both together

% arm displacement 
%x_bm = 1 + E_ * 0.5 * (-0.136*i_ -0.068*j_ -0.2756*k_);
%base.set_frame_displacement(x_bm);

% coupling
robot = DQ_WholeBody(base);
robot.add(lwr4);

%% controller

% controller
x = {};
e = ones (8 ,1);
e_norm = [];

t_t = 0;
t_i = 1;
while norm(e(:,end)) > error_tol

    %% extracting jacobians
    
    % obtains the jacobian relating joints position and
    % end-effector dual quaternion terms velocities
    J_robot = robot.pose_jacobian(q);
    
    % obtains the mobile base jacobian, relating wheels velocities to the
    % configuration velocities
    J_base = constraint_jacobian_custom(q, wheel_radius, wheels_side_distance);
    
    %% updating control
    
    % obtains the end-effector pose state given the joints state
    x{t_i} = robot.fkm(q);
    
    % directly computes error by subtracting the current pose dual
    % quaternion by the desired pose dual quaternion
    e(:,t_i) = vec8(x{end}-xd);
    
    % traditionally computes the control signal given the jacobian, gain
    % and computed error
    u = -pinv(J_robot)* gain * e(:,end);
    
    %% control signal integration
    
    % mapping wheels speed to configuration space velocities
    u_m = [J_base*u(1:2); u(3:end)];
        
    % integrates the control signal to the joints states
    q = q + T*u_m;
    
    %% after computations
        
    % saving auxiliary values 
    e_norm(t_i) = norm(e(:,end));

    % saving current time
    if t_i > 1
        t_t(t_i) = t_t(t_i-1) + T*t_i
    else
        t_t(t_i) = T*t_i;
    end
    
    % updating iterative variable
    t_i = t_i + 1;

end


% plotting result
plot(e_norm);

figure
for i=1:length(x)
    
    plot(x{i});
    hold on;
    plot(xd);
    hold off;
    pause(0.0001);
end

