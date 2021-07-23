function cmd_clp = rosi_clip_lever_pos(cmd_in, min, max)
% CLIPS ROSI lever arms joints position
    
    cmd_clp = [];
    for i=1:length(cmd_in)
        if cmd_in(i) < min
            cmd_clp(i) = min;
        elseif cmd_in(i) > max
            cmd_clp(i) = max;
        else
            cmd_clp(i) = cmd_in(i);
        end
    end
    
end

