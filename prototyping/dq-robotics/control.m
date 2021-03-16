clear all;
close all;

%% preamble

% for using directly Dual Quaternion library namespace
include_namespace_dq;

% loading kuka robot
lwr4 = KukaLwr4Robot.kinematics();

%% parameters

% declaring set-point
r = cos(pi /2) + j_*sin(pi /2);
p = 0.1* i_ + 0.2* j_ + 0.3* k_;
xd = r + E_ *0.5* p*r;

% setting current joints position
q = [0, 0.3770 , 0.1257 , -0.5655 , 0, 0, 0].';

% sampling time
T = 0.001;

% controller gain
gain = 10;

% error tolerance
error_tol = 0.01;

%% controller

% controller
x = {};
e = ones (8 ,1);
e_norm = [];

t_t = 0;
t_i = 1;
while norm(e(:,end)) > error_tol

    % updating control
    J = lwr4.pose_jacobian(q);
    x{t_i} = lwr4.fkm(q);
    e(:,t_i) = vec8(x{end}-xd);
    u = -pinv(J)* gain*e(:,end);
    q = q + T*u;

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

% figure
% for i=1:length(x)
%     
%     plot(x{i});
%     hold on;
%     plot(xd);
%     hold off;
%     pause(0.0001);
% end

