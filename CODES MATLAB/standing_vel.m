%#####################################################################################################
%# Function : To calculate the velocities during standinga and walking                               #   
%#                                                                                                   #       
%# Input(s)  : dq(angular velocities)                                                                #
%#                                                                                                   #                
%# Ouptut(s) : v(linear velocities), w(angular velocities)                                           # 
%#                                                                                                   #       
%# Example: None                                                                                     #   
%#                                                                                                   #       
%#                                                                                                   #               
%#                                                                                                   #                                       
%#####################################################################################################

function [v,w] = standing_vel(dq)
global robot;
global base;
global change;

v = zeros(3,29);
w = zeros(3,29);

if (base == 'hips')
    if(change == 1)
        for i = 28:-1:23
            [v(:,i),w(:,i)] = calVW(i,robot.parts(i).C_Id,dq,1);
        end
        [v(:,1),w(:,1)] = calVW(1,23,dq,1);
        
        for i = 2:15
            [v(:,i),w(:,i)] = calVW(robot.parts(i).P_Id,i,dq,0);
        end
        
        for i = 16:22
            [v(:,i),w(:,i)] = calVW(robot.parts(i).P_Id,i,dq,0);
        end
    end
    
    if(change == 0)
        for i = 21:-1:16
            [v(:,i),w(:,i)] = calVW(i,robot.parts(i).C_Id,dq,1);
        end
        
        [v(:,1),w(:,1)] = calVW(1,16,dq,1);
        
        for i = 2:15
            [v(:,i),w(:,i)] = calVW(robot.parts(i).P_Id,i,dq,0);
        end
        
        for i = 23:29
            [v(:,i),w(:,i)] = calVW(robot.parts(i).P_Id,i,dq,0);
        end
    end
    
    
else
    
    for i = 1:21
        [v(:,i),w(:,i)] = calVW(robot.parts(i).P_Id,i,dq,0);
    end
    
    for i = 27:-1:22
        [v(:,i),w(:,i)] = calVW(i,robot.parts(i).C_Id,dq,1);
    end
    
    [v(:,7),w(:,7)] = calVW(7,22,dq,1);
    
end

end
