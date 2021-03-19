function ros_send_robot_cmd_vel(u, pub_traction_speed, pub_manipulator_speed)

    % treating header
    time = rostime("now");
    header = rosmessage('std_msgs/Header');
    header.Stamp.Sec = time.Sec;
    header.Stamp.Nsec = time.Nsec;
    header.FrameId = 'rosi';

    % mounting traction message
    msg_traction_vel = rosmessage(pub_traction_speed);
    msg_traction_vel.Header = rosmessage('std_msgs/Header');
    msg_traction_vel.Header = header;
    
    aux_blankMessage = rosmessage("sim_rosi/RosiMovement");
    aux_msgArray = [aux_blankMessage, aux_blankMessage, aux_blankMessage, aux_blankMessage];
    for i=1:4 
        
        aux_Msg = rosmessage("sim_rosi/RosiMovement");
        
        aux_Msg.NodeID = i;
        
        if i <=2
            aux_Msg.JointVar = u(1);
        else
            aux_Msg.JointVar = u(2);
        end
        
        aux_msgArray(i) = aux_Msg;
    end
    
    msg_traction_vel.MovementArray = aux_msgArray;
    
    % mounting manipulator message
    msg_manipulator_vel = rosmessage(pub_manipulator_speed);
    msg_manipulator_vel.Header = header;
    msg_manipulator_vel.JointVariable = [u(3:end)];
    

    %pub_traction_speed.
    send(pub_traction_speed, msg_traction_vel);
    send(pub_manipulator_speed, msg_manipulator_vel);
end

