function cmd_clp = rosi_clip_lever_pos(cmd_in)
% CLIPS ROSI lever arms joints position
    
    % defining clipping angles
    angle_max = deg2rad(165);
    angle_min = deg2rad(15);
    
    cmd_clp = [];
    for i=1:length(cmd_in)
        if cmd_in(i) < angle_min
            cmd_clp(i) = angle_min;
        elseif cmd_in(i) > angle_max
            cmd_clp(i) = angle_max;
        else
            cmd_clp(i) = cmd_in(i);
        end
    end
    
end

