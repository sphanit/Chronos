%#####################################################################################################
%# Function : To calculate the inverse dynamics                                                      #   
%#                                                                                                   #       
%# Input(s)  : n                                                                                     #
%#                                                                                                   #                
%# Ouptut(s) : updates the global model                                                              # 
%#                                                                                                   #       
%# Example: None                                                                                     #   
%#                                                                                                   #       
%#                                                                                                   #               
%#                                                                                                   #                                       
%#####################################################################################################


function [f,t,tt] = InvDyn(n)

global robot;
global ph;
global temp;
global temp2;
global tm;

if(ph == 'ds')
    if(n==0)
        f = [0;0;0];
        t = [0;0;0];
        tt = 0;
        return;
    end
    
    % CoM and Inertia in Global frame
    c = robot.parts(n).com_g;
    I = robot.parts(n).R_mat*robot.parts(n).I*robot.parts(n).R_mat';
    m = robot.parts(n).mass;
    c_hat = hat(c);
    I = I + m * c_hat * c_hat';
    
    P = m * robot.parts(n).v0 + cross(robot.parts(n).w0,c');
    L = m * cross(c',robot.parts(n).v0) + I * robot.parts(n).w0;
    
    if(n~=0)
        f_ext = m * [-98000;0;0];
        t_ext = m * cross((robot.parts(n).com_g' - robot.parts(n).joint_loc),[-98000;0;0]);
    else
        f_ext = 0;
        t_ext = 0;
    end
    tt = 1;
    
        
    f = m * (robot.parts(n).dv0 + cross(robot.parts(n).dw,c')) + cross(robot.parts(n).w0,P) - f_ext;
    t = m * (cross(c',robot.parts(n).dv0)) + I * robot.parts(n).dw + cross(robot.parts(n).v0,P) + cross(robot.parts(n).w0,L) - t_ext;
    
    robot.parts(n).P0 = P;
    robot.parts(n).L0 = L;
    robot.parts(n).f1 = f;
    robot.parts(n).t1 = t;
    robot.parts(n).tt1 = tt;
    if(length(robot.parts(n).C_Id) == 2)
        [f1,t1,tt1] = InvDyn(robot.parts(n).C_Id(1));
        f = f + f1;
        t = t + t1;
        tt = tt + tt1;
        if(n ~=7)
            [f2,t2,tt2] = InvDyn(robot.parts(n).C_Id(2));
            f = f + f2;
            t = t + t2;
            tt = tt + tt2;
        else
            InvDyn(robot.parts(n).C_Id(2));
        end
    else
        if(n<=21)
            [f1,t1,tt1] = InvDyn(robot.parts(n).C_Id);
            f = f + f1;
            t = t + t1;
            tt = tt + tt1;
        else
            if(size(temp)~=0)
                f = temp + f;
                temp = f;
                t = temp2 + t;
                temp2 = t;
                tt = tm + tt;
                tm = tt;
            end
            InvDyn(robot.parts(n).C_Id);
        end
    end
    
    if (n == 7)
        robot.parts(n).tt0 = tt;
        robot.parts(n).f0 = f;
        robot.parts(n).t0 = t;
        
        %Calculate torque here
        
        robot.parts(n).u = robot.parts(n).sv' * f + robot.parts(n).sw' * t;
        
        f = f/2;
        t = t/2;
        tt = tt/2;
        temp = f;
        temp2 = t;
        tm = tt;
    end
    if(n~=7)
        robot.parts(n).f0 = f;
        robot.parts(n).t0 = t;
        robot.parts(n).tt0 = tt;
        
        %Calculate torque here
        
        robot.parts(n).u = robot.parts(n).sv' * f + robot.parts(n).sw' * t;
        
    end
    
    
    
elseif(ph == 'ss')
    if(n==0)
        f = [0;0;0];
        t = [0;0;0];
        return;
    end
    
    % CoM and Inertia in Global frame
    c = robot.parts(n).com_g;
    I = robot.parts(n).R_mat*robot.parts(n).I*robot.parts(n).R_mat';
    m = robot.parts(n).mass;
    c_hat = hat(c);
    I = I + m * c_hat * c_hat';
    
    
    
    if(length(robot.parts(n).C_Id) == 2)
        ForwKin_Dyn(robot.parts(n).C_Id(1));
        f = f + f1
        ForwKin_Dyn(robot.parts(n).C_Id(2));
        f = f+f2
    else
        ForwKin_Dyn(robot.parts(n).C_Id);
        f = f+f1
    end
    u = f.sw
end

end

