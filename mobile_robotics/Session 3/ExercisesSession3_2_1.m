%% Parameters
currentState='findWall';
v=$;
r=$;
omega=$;
while true
    $$$ % update all needed sensors
    pause(0.015);
    switch currentState
        
        case 'findWall'
             $$$ % actuators   
             if $$$ % state switch possibility                  
                currentState=$$$; % if true, apply this state next cycle
             else
                currentState=$$$; % else, apply this state next cycle
             end
        
        case 'turnCCW'
             $$$
             if ???
                currentState='???';
             else
                currentState='???';
             end
        
        case 'goForward'
            $$$
            if $$$
                currentState='???';
            elseif $$$
                currentState='???';
            else
               currentState='???';
            end
            
        case 'lostWall'
            $$$
            if $$$
                currentState='???';
            elseif $$$
                currentState='???';
            else
                currentState='???';
            end
            
        otherwise
            turtlebot.stop()
            disp('error, unknown state, break'); % this will never happen...
            break;
    end
end
end
