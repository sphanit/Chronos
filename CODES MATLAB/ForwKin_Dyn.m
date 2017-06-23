%#####################################################################################################
%# Function  : updates kinematics for the calculation of dynamics                                    #   
%#                                                                                                   #       
%# Input(s)  : n(part_Id,1)                                                                          #
%#                                                                                                   #                
%# Ouptut(s) : updates the model                                                                     # 
%#                                                                                                   #       
%# Example: ForwKin_Dyn(1)                                                                           #   
%#                                                                                                   #       
%#                                                                                                   #               
%#                                                                                                   #                                       
%#####################################################################################################


function [] = ForwKin_Dyn(n)
global robot

if(n == 0)
    return;
end

if(n~=1)
%     disp(n);
    
    pid = robot.parts(n).P_Id;
    
    %Position and oreinetation
    p = robot.parts(n).axis_loc;
    
    % Spatial Velocity
    sw = robot.parts(pid).R_mat*robot.parts(n).a;
    sv = cross(p,sw);
    robot.parts(n).w0 = robot.parts(pid).w0 + sw*robot.parts(n).dq;
    robot.parts(n).v0 = robot.parts(pid).v0 + sv*robot.parts(n).dq;
    
    % Spatial Acceleration
    dsv = cross(robot.parts(pid).w0,sv) + cross(robot.parts(pid).v0,sw);
    dsw = cross(robot.parts(pid).w0,sw);
    robot.parts(n).dw = robot.parts(pid).dw + dsw*robot.parts(n).dq + sw*robot.parts(n).ddq;
    robot.parts(n).dv0 = robot.parts(pid).dv0 + dsv*robot.parts(n).dq + sv*robot.parts(n).ddq;
    robot.parts(n).sw = sw;
    robot.parts(n).sv = sv;
    
elseif(n == 1)
%     disp(n);
    
    %Position and oreinetation
    p = robot.parts(n).axis_loc;
    
    % Spatial Velocity
    sw = robot.parts(n).a;
    sv = cross(p,sw);
    robot.parts(n).w0 = sw*robot.parts(n).dq;
    robot.parts(n).v0 = sv*robot.parts(n).dq;
    
    % Spatial Acceleration
    dsv = [0;0;0];
    dsw = [0;0;0];
    
    robot.parts(n).dw  = dsw*robot.parts(n).dq + sw*robot.parts(n).ddq;
    robot.parts(n).dv0 = dsv*robot.parts(n).dq + sv*robot.parts(n).ddq;
    robot.parts(n).sw  = sw;
    robot.parts(n).sv  = sv;
    
end

if(length(robot.parts(n).C_Id) == 2)
    ForwKin_Dyn(robot.parts(n).C_Id(1));
    ForwKin_Dyn(robot.parts(n).C_Id(2));
else
    ForwKin_Dyn(robot.parts(n).C_Id);
end

end
