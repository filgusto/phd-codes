clear all;
clc;
close all;

addpath('./lib');

%% Parameters

% number of transform elements
n_qtd = 100;
n = -n_qtd:n_qtd;

% points within 1s 
t_q = 100;

% time array
t = linspace(0,1,t_q);

% defining input function
f_in = @f_xsquare;


%% Computing c_n

% computing each c
i = 1;
for i = 1 : length(n);
    
    % compute each c_n
    c_n(i,1) = compute_cn(f_in, t, n(i));
end


%% Computing the Fourier transform output for the time sequence

for i = 1 : length(t)   
    for k = 1 : length(n)
        % computing each c_n element for each timestep
        y(i,k) = c_n(k) .* exp(n(k)*2*pi*1i*t(i));  
    end 
end

% Summin each timestep for plotting
y_sum = sum(y,2);

%% Plotting the result

figure;
hold on;
grid on;
for i = 1 : length(t)
    
    % computing the actual function value
    f_in_y = f_in(t(i));
    
    % plotting the actual function
    plot(real(f_in_y), imag(f_in_y), '*k');
    
    % plotting the fourier estimative
    plot(real(y_sum(i)), imag(y_sum(i)), '*r');
    
    pause(0.2);
    
end

xlabel('x axis');
ylabel('y axis');
title('Fourier transform output');
legend('actual function', 'Fourier estimative');




