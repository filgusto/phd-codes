function teleoperate_differential_drive_robot(varargin)

    %% Configuring the nonholonomic mobile base
    if nargin == 0
        wheel_radius = 0.1; % Wheel radius (meters)
        distance_between_wheels = 0.4; % Distance between wheels (meters)
    elseif nargin == 2
        wheel_radius = varargin{1};
        distance_between_wheels = varargin{2};
    else
        error_msg = sprintf(...
        ['\nUsage: \n'...        
        'teleoperate_differential_drive_robot() starts the teleoperation of'... 
        ' the simulated robot with default parameters; that is, wheel radius'...
        ' equal to 0.1 m and distance between wheels equal to 0.4 m.\n\n'...
        'teleoperate_differential_drive_robot(wheel_radius,'...
        ' distance_between_wheels) start the teleoperation of the simulated'...
        ' robot with parameters given by wheel_radius and '...
        'distance_between_wheels, both in meters.']); 
        error(error_msg);
    end

    base = DQ_DifferentialDriveRobot(wheel_radius, distance_between_wheels);
    
    fprintf(['\nStarting simulation of a differential drive robot with wheel'...
        'radius equal to %f m and distance between wheels equal to %f m'], ...
        wheel_radius, distance_between_wheels);

    q = [0,0,0]'; % Initial configuration [pos x, pos y, ang z]
    T = 0.001; % Integration step for the animation

    %% Scene setup
    %Key-press events are handled by the function keypress.
    fig_handle = figure('KeyPressFcn',@keypress);
    hold on;
    axis equal;
    axis(distance_between_wheels*[-10,10,-10,10,0,1/distance_between_wheels]);    
    view(3);
    xlabel('X');
    ylabel('Y');
    title(['Press ''q'' to quit. Use the keyboard arrows to teleoperate the '...
        'robot.']);


    % The struct S is shared between the main function and the keypress
    % function.
    S.u = [0;0]; % Velocity inputs. The robot is intially stopped
    S.vel = 100; % Wheels angular velocity step
    S.quit = '@'; % The program quits when this variable equals '@'

    %Store the struct in the figure
    guidata(fig_handle,S);

    %% Robot teleoperation with the keyboard
    while S.quit ~= 'q'
        % Get the information updated by the keypress() function
        S = guidata(fig_handle);
        % update robot configuration and draw it
        S.u
        q = q + T * base.constraint_jacobian(q(3))*S.u;
        plot(base,q);
        drawnow;
    end

    %% Time to quit
    close(fig_handle);
    clc;   
    disp('That''s it, I''m out!');
    clear fig_handle;
    return;
end