%#####################################################################################################
%# Function : To calculate the inverse kinematics on COM                                              #   
%#                                                                                                   #       
%# Input(s)  : x,y,z -COM trajectories, Tr_mat(Transformation matrix),enf(end effectors)             #
%#               from (start link Id),com(COM of the robot),theta                                    #
%#                                                                                                   #                
%# Ouptut(s) :flag(if 0, solution doesnt exist and vice-versa),q(angles matrix),gg(end effector      # 
%#             axis location),pts(points on the trajectory)                                          #       
%# Example:                                                                                          #   
%#                                                                                                   #       
%#                                                                                                   #               
%#                                                                                                   #                                       
%#####################################################################################################


function [flag,q,gg,pts] = InvKin_com(x,y,z,tg,Tr_mat,enf,from,theta,com)
global robot;
global base;
global count;
%global move_base;
%global steps;

disp(count);
pause(1)

% Tr_mat = eye(4);

dq = zeros(1,28);

J_tmp = zeros(6,28);

Idx = find_route(enf,from);

l = length(Idx);

[x1,y1,z1]  = LegTraj(robot.parts(1).axis_loc,1,142.1,10,0,20);

x1(21) = x1(20);
y1(21) = y1(20);
z1(21) = z1(20);

for ang=1:length(x)
    
    if (count == 1)
        Tr_mat = [1 0 0 x1(ang);
            0 1 0 y1(ang);
            0 0 1 z1(ang);
            0 0 0 1];
    end
    
    %com = ForwKin(theta,Tr_mat,dq);

    t(ang,:) = [x(ang),y(ang),z(ang)];
    g = tg(:,ang);
    disp(ang);
    
    sdq = zeros(1,28);
    flag = 0;
    
    for i=1:500
       % fprintf('it = %d\n',i);
        
        err = t(ang,:) - [com.x,com.y,com.z];
        err1 = g - robot.parts(enf).axis_loc;
        
        collision_data();
        
        if (norm(err) < 0.001 && norm(err1) < 0.001)
            q(:,ang) = theta;
            Dq(:,ang) = rad2deg(sdq);
            pos(:,ang) = com;
            flag = 1;
            gg = robot.parts(enf).axis_loc;
            pts(:,:,ang) = [robot.draw];
            %sample_draw(pts)
            break;
            
        else
            u = [robot.parts(Idx).Z_w];
            p = [robot.parts(Idx).axis_loc];
            J = Jacob(u,p);
            J_tmp(:,Idx(1:l-1)) = J(:,:);
            
            J_g = Jacob_CoM ();
            J_aug = [J_g;J_tmp(1:3,:)];
            
            J_Inv = invsvd_lds(J_aug,0.1);
            dq = J_Inv*[err';err1];
            
            theta = theta + rad2deg(dq)';
            
            
             Angle_limit();
%              theta(12:15) = 0;
%              theta(17:20) = 0;
              theta(7:11) = 0;
            com = ForwKin(theta,Tr_mat,rad2deg(dq)');
            
        end
    end
    
    if(flag==0)
        disp('Solution Does not Exist');
        q(:,ang) = theta;
        break;
        
    end
end


    function [] = Angle_limit()
        ang_limits = limits_2(base);
        theta_lim_min = ang_limits(1,:);
        theta_lim_max = ang_limits(2,:);
        len = 28;
        for idx = 1:(len-1)
            if(theta(idx)) < theta_lim_min(idx)
                theta(idx) = theta_lim_min(idx);
            elseif(theta(idx) > theta_lim_max(idx))
                theta(idx) = theta_lim_max(idx);
            end
        end
        
    end

function [stable] = stability_check(ang)
         
        stable = 0;
        
        if(ang < 101)
            stable = 1;
        else
            dP = P-P_pre;
            dL = L-L_pre;
            pz = (4000*9800*com.z - dL(2,:))/(4000*9800+dP(1,:));
            py = (4000*9800*com.y + dL(3,:))/(4000*9800+dP(1,:));
            
            if change == 1
                f1 = robot.parts(29).axis_loc;
                f1_z = [f1(3)+60,f1(3)-90,f1(3)-90,f1(3)+60,f1(3)+60];
                f1_y = [f1(2)-23,f1(2)-23,f1(2)+23,f1(2)+23,f1(2)-23];
                
                figure(2);
                plot(pz,py,'ko');
                hold on,plot(f1_z,f1_y);
                hold off;
                stable = inpolygon(pz,py,f1_z,f1_y);
            else
                f1 = robot.parts(22).axis_loc;
                f1_z = [f1(3)-60,f1(3)+90,f1(3)+90,f1(3)-60,f1(3)-60];
                f1_y = [f1(2)-23,f1(2)-23,f1(2)+23,f1(2)+23,f1(2)-23];
                
                stable = inpolygon(pz,py,f1_z,f1_y);
                
            end
        end
        
    end
end