function ret = sig(v_in)    
% Returns the signal of a variable
    if v_in >= 0
        ret = 1;
    else
        ret = -1;        
end

