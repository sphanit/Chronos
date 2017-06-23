%#####################################################################################################
%# Function :To check and obtain the inverse kinematics solution to reach one link from some other   #   
%#            link                                                                                   #       
%# Input(s)  :x1,y1,z1,x2,y2,z2 - trajectories;part_Id1;part_Id2;from(starting link Id),adj(adjacent)#
%#            dq(angular velocities),theta                                                           #                
%# Ouptut(s) : flag(if 0, then the solution doesntexist and vice-versa),q(angles matrix),            # 
%#             Dq(angular velocities),pos(position)                                                  #       
%# Example: InvKin_mult(x1,y1,z1,4,x2,y2,z2,5,1,2,adj,theta,dq)                                      #   
%#                                                                                                   #       
%#                                                                                                   #               
%#                                                                                                   #                                       
%#####################################################################################################


function [flag,q,Dq,pos] = InvKin_mult(x1,y1,z1,part_Id1,x2,y2,z2,part_Id2,from,from2,adj,theta,dq)
global robot;
global base;
global count;
global steps;
global change;
global move_base;
%theta = zeros(1,28);

count = count + 1;

Idx1 = find_route(part_Id1,from)
Idx2 = find_route(part_Id2,from2)
Idx = [Idx1,Idx2];
Idx = unique(Idx);
len = length(Idx);
I = eye(len-1);
[tp1,tp2,x_h,z_h] = new_traj(2*steps-1);
%y_h = 20*sin(2*pi.*[1:100]/200);
tmp = theta;
tmp2 = dq;
P_pre = 0;
L_pre = 0;
J_tmp = zeros(6,28);
J_tmp1 = zeros(6,28);
for ang=1:length(x1)
    if(move_base == 1)
        Tr_mat = [1 0 0 x_h(ang+count*length(x1));
            0 1 0 0;
            0 0 1 z_h(ang+count*length(x1));
            0 0 0 1];
    else
        Tr_mat = eye(4);
    end
    
     [com,cm,P,L] =ForwKin(theta,Tr_mat,deg2rad(dq));
    
    t1(:,ang) = [x1(ang);y1(ang);z1(ang)];
    t2(:,ang) = [x2(ang);y2(ang);z2(ang)];
    disp(ang);
    sdq1 = zeros(1,length(Idx1)-1);
    sdq2 = zeros(1,length(Idx2)-1);
    flag = 0;
    for i=1:500
        pre1 = robot.parts(Idx1(length(Idx1))).axis_loc;
        err1 = t1(:,ang)-pre1;
        pre2 = robot.parts(Idx2(length(Idx2))).axis_loc;
        err2 = t2(:,ang)-pre2;
        collision_data();
        
        %if ( abs(err1(1)) < 0.001 && abs(err1(2)) < 0.001 && abs(err1(3)) < 0.001 && abs(err2(1)) < 0.001 && abs(err2(2)) < 0.001 && abs(err2(3)) < 0.001 && collision_check)
        if(norm(err1)<0.001 && norm(err2)<0.001  && stability_check(ang) && collision_check)
         %   ForwDyn
             InvDyn(1);
           J_tau = robot.parts(7).u*(10^-9)
            q(:,ang) = theta;
            tmp = theta;
            tmp2 = dq;
            P_pre = P;
            L_pre = L;
            Dq(:,ang) = dq;
            pos(:,ang) = [pre1;pre2];
            pre1;
            flag = 1;
            break;
            
        else
            u1 = [robot.parts(Idx1).Z_w];
            p1 = [robot.parts(Idx1).axis_loc];
            J1 = Jacob(u1,p1);
            u2 = [robot.parts(Idx2).Z_w];
            p2 = [robot.parts(Idx2).axis_loc];
            J2 = Jacob(u2,p2);
            
            J_tmp(:,Idx1(1:length(Idx1)-1)) = J1(:,:);
            J_tmp1(:,Idx2(1:length(Idx2)-1)) = J2(:,:);
            
            J_aug = [J_tmp;J_tmp1];
            
            J_Inv = invsvd_lds(J_aug,0.1);
            dq = J_Inv*[err1;0;0;0;err2;0;0;0];
            theta = theta + rad2deg(dq)';
            
            Angle_limit();
            %theta(7)=0;
            %theta(11)=0;
            
            dq = theta-tmp;
            ddq = dq-tmp2;
            
            for up = 1:28
                robot.parts(up).dq  = dq(up); 
                robot.parts(up).ddq = ddq(up);
            end
            
            [com,cm,P,L] = ForwKin(theta,Tr_mat,deg2rad(dq));
            ForwKin_Dyn(1);
        end
    end
    if(flag==0)
        disp('Solution Does not Exist');
        q(:,ang) = theta;
        Dq(:,ang) = rad2deg([sdq1,sdq2]);
        pos(:,ang) = [pre1;pre2];
        
        break;
    end
    if(ang == length(x1))
        ForwKin(theta,Tr_mat,deg2rad(dq));
        disp('hahahahhahahhahahahahahahahahahahahahhahahahahaha');
    end
end


    function [] = Angle_limit()
        ang_limits = limits_2(base);
        theta_lim_min = ang_limits(1,:);
        theta_lim_max = ang_limits(2,:);
        for idx = 1:(len-1)
            if(theta(Idx(idx)) < theta_lim_min(Idx(idx)))
                theta(Idx(idx)) = theta_lim_min(Idx(idx));
            elseif(theta(Idx(idx)) > theta_lim_max(Idx(idx)))
                theta(Idx(idx)) = theta_lim_max(Idx(idx));
            end
        end
        
    end

    function [stable] = stability_check(ang)
        fprintf('Enter = %d\n',ang);
        if(ang < 1)
            stable = 1;
        else
            dP = P-P_pre;
            
            dL = L-L_pre;
            pz = (4000*98000*com.z - dL(2,:))/(4000*98000+dP(1,:));
            py = (4000*98000*com.y + dL(3,:))/(4000*98000+dP(1,:));
            
            f1 = robot.parts(1).axis_loc;
            f1_z = [f1(3)+60,f1(3)-90,f1(3)-90,f1(3)+60,f1(3)+60];
            f1_y = [f1(2)-23,f1(2)-23,f1(2)+23,f1(2)+23,f1(2)-23];
            
            f2 = robot.parts(28).axis_loc;
            f2_z = [f2(3)+60,f2(3)-90,f2(3)-90,f2(3)+60,f2(3)+60];
            f2_y = [f2(2)-23,f2(2)-23,f2(2)+23,f2(2)+23,f2(2)-23];
            figure(1);
            hold on;
            plot([f2_z(1),f2_z(2),f1_z(3),f1_z(4),f2_z(1)],[f2_y(1),f2_y(2),f1_y(3),f1_y(4),f2_y(1)]);
            plot(pz,py,'ko');
            plot(com.z,com.y,'ro');
            hold off;
            %disp(com);
            stable = inpolygon(pz,py,[f2_z(1),f2_z(2),f1_z(3),f1_z(4),f2_z(1)],[f2_y(1),f2_y(2),f1_y(3),f1_y(4),f2_y(1)]);
        end
        
    end
end

