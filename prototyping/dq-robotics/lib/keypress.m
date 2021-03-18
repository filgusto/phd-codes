% keypress() handles the keyboard events
function keypress(H,E)
    S = guidata(H);
    
    %Self-explanatory state machine.
    switch E.Key
        case 'rightarrow'
            S.u = S.u + [-S.vel;S.vel];

        case 'leftarrow'
            S.u = S.u - [-S.vel;S.vel];

        case 'uparrow'
            S.u = S.u + S.vel;

        case 'downarrow'
            S.u = S.u - S.vel;
            
        case 'q'
            S.quit = 'q';
    end
    
    % Store the updated control inputs and the program status (S.quit)
    guidata(H,S);
end