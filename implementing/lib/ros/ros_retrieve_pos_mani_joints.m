% Retrieves manipulator joints angular position and returns a vector
function q = ros_retrieve_pos_mani_joints(sub_pos_mani_joints)

    % collects ros_messages
    data_mani = receive(sub_pos_mani_joints, 3);
    
    % extracting manipulator data
    q = data_mani.JointVariable;

end

