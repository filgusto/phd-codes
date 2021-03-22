function ros_gentle_init()

    try 
        rosinit;
    catch
        disp('Rosinit already initialized.');
    end

end

