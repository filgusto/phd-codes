function fcn_publisher(pub_topic)

    
    while true
        
        % prepares a publishing message
        pub_msg = rosmessage(pub_topic);
        time = rostime("now");
        pub_msg.Stamp.Sec = time.Sec;
        pub_msg.Stamp.Nsec = time.Nsec;
        pub_msg.FrameId = 'matlab';
        
        disp(pub_msg.Stamp.Sec);
        
        % publishes the message
        send(pub_topic, pub_msg);
    end
    
   
end

