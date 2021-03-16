function q = ros_retrieve_joints(sub_trac, sub_mani)

    % collects a ros_message
    topic_data_trac = receive(sub_trac, 3);
    topic_data_mani = receive(sub_mani, 3);

    % extracting traction data
    data_trac_aux = topic_data_trac.MovementArray;
    q_trac = zeros(4,1);
    for i=1:4
        q_trac(i,1) = data_trac_aux(i).JointVar;
    end

    % extracting manipulator data
    q_mani = topic_data_mani.JointVariable;

    % stacking values
    % q_trac(1): front-right traction joint
    % q_trac(3): front-left  traction joint
    q = [q_trac(1); q_trac(1); q_mani];

end

