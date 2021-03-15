function fcn_callback_example(sub_handler, msg) 
    % you may use global variables to pass data among functions
    disp(msg.JointVariable);
end

