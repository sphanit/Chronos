%#####################################################################################################
%# Function : To calculate the inverse kinematics on any link                                        #   
%#                                                                                                   #       
%# Input(s)  : x,y,z -trajectories,parts,from (start link Id),adj(adjustment needed to getproper     #
%#             theta values)                                                                         #                
%# Ouptut(s) : flag(0 if solution doesnt exist and vice-versa),q(anles matrix)                                                              # 
%#                                                                                                   #       
%# Example: InvKin_mult2(x,y,z,parts,1,0.01)                                                                                     #   
%#                                                                                                   #       
%#                                                                                                   #               
%#                                                                                                   #                                       
%#####################################################################################################

function [flag,q] = InvKin_mult2(x,y,z,parts,from,adj)
global robot;
global base;
global count;
global move_base;
global steps;

count = count + 1;
theta = zeros(1,28);

n = length(parts)

if(~isempty(adj))
    theta(adj(:,:)) = 20;
end

Idx = [];
for i=1:n
    Idx_p{i} = find_route(parts(i),from(i));
    Idx = [Idx, Idx_p{i}];
end
Idx = unique(Idx);
len = length(Idx);
I = eye(len-1);

for i=1:n
    J_tmp{i} = zeros(6,28);
end


for ang=1:length(x)
    if(move_base == 1)
        Tr_mat = ones(4);
    else
        Tr_mat = eye(4);
    end
    
    %ForwKin(theta,Tr_mat,zeros(1,28));
    
    t(:,:,ang) = [x(:,ang),y(:,ang),z(:,ang)];
    disp(ang);
    
    flag = 0;
    
    
    for i=1:2000
        
        J_aug = [];
        total_err = [];
        err_flag = 1;
        
        for j = 1:n
            last = length(Idx_p{j});
            pre(:,j) = robot.parts(Idx_p{j}(last)).axis_loc;
            err(:,j) = t(j,:,ang)'-pre(:,j);
            
            if(norm(err(:,j)) > 0.001)
                err_flag = 0;
            end
        end
        
        if (err_flag)
            
            q(:,ang) = theta;
            
            flag = 1;
            
            break;
            
        else
            
            for j = 1:n
                u{j} = [robot.parts(Idx_p{j}).Z_w];
                p{j} = [robot.parts(Idx_p{j}).axis_loc];
                
                J{j} = Jacob(u{j},p{j});
                
                J_tmp{j}(:,Idx_p{j}(1:length(Idx_p{j})-1)) = J{j};
                
                J_aug = [J_aug;J_tmp{j}];
                total_err = [total_err;err(:,j);0;0;0];
            end
            
            J_Inv = invsvd_lds(J_aug,0.1);
            dq = J_Inv*total_err;
            
            theta = theta + rad2deg(double(dq'));
            
            Angle_limit();
            ForwKin(theta,Tr_mat,zeros(1,28));
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
        for idx = 1:(len-1)
            if(theta(Idx(idx)) < theta_lim_min(Idx(idx)))
                theta(Idx(idx)) = theta_lim_min(Idx(idx));
            elseif(theta(Idx(idx)) > theta_lim_max(Idx(idx)))
                theta(Idx(idx)) = theta_lim_max(Idx(idx));
            end
        end
        
    end
    function [stable] = stability_check()
        accy = 0;
        accz = 0;
        
        %%Assuming Cart-Table Model
        y_zmp = com.y - accy*com.x/9.8;
        z_zmp = com.z - accz*com.x/9.8;
        
        zlim = [-65.5,90.5];
        ylim = [-182.01,37.41];
        
        if(y_zmp >= ylim(1) && y_zmp <= ylim(2) && z_zmp >= zlim(1) && z_zmp <= zlim(2))
            stable = true;
        else
            stable = false;
        end
        
    end
end


