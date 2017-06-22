
function [flag,q,Dq,pos,cdl,HG] = InvKin(x,y,z,part_Id,from,adj,theta)
global robot;
global base;
global count;
global move_base
count = count + 1;
% theta = zeros(1,28);
% theta(adj) = -20;
Idx = find_route(part_Id,from);
len = length(Idx);
I = eye(len-1);

if(move_base ==1)
    [t1,t2,x_h,z_h] = new_traj;
end

cdl = [];
P_pre = 0;
L_pre = 0;
sdel_q = zeros(28,1);
for ang=1:length(x)
    
    if(move_base==1)
        Tr_mat = [1 0 0 x_h(ang+count*length(x));
            0 1 0 0;
            0 0 1 -z_h(ang+count*length(x));
            0 0 0 1];
    else
        Tr_mat = eye(4);
    end
    [com,cm,P,L] = ForwKin(theta,Tr_mat,zeros(1,28));
    cdl= [cdl,com];
    t(:,ang) = [x(ang);y(ang);z(ang)];
    %     disp(ang);
    sdq = zeros(1,len-1);
    flag = 0;
    for i=1:500
        pre = robot.parts(Idx(len)).axis_loc;
        err = t(:,ang)-pre;
        collision_data();
        if(collision_check==0)
            break;
        end
        if ( norm(err)<0.001  && collision_check && stability_check(ang))
            [h,AG,hG] = M_mats(Tr_mat,com,sdel_q);
            N = 304.3457*com.mass*1;
            HG = abs(hG(1:3))/N;
            q(:,ang) = theta;
            P_pre = P;
            L_pre = L;
            cdl= [cdl,com];
            Dq(:,ang) = sdel_q;
            pos(:,ang) = pre;
            flag = 1;
            break;
            
        else
            u = [robot.parts(Idx).Z_w];
            p = [robot.parts(Idx).axis_loc];
            J1 = Jacob(u,p);
            J2 = invsvd_lds(J1,0.5);
            dq = J2*[err;0;0;0];
            sdq = sdq + (double(dq'));
            theta(Idx(1:len-1))= theta(Idx(1:len-1))+rad2deg(double(dq'));
            Angle_limit();
            %           theta(8:11) = 0;
            
            
            sdel_q = zeros(28,1);
            del_q = zeros(28,1);
            
            sdel_q(Idx(1:len-1)) = (sdq);
            del_q(Idx(1:len-1)) = (dq);
            
            [com,cm,P,L] = ForwKin(theta,Tr_mat,del_q);
            
            [h,AG,hG,vG] = M_mats(Tr_mat,com,del_q);
            seq = gen_rand_num([1:3],9);
            ths = [7,8,9,10,11,17,18,19,20];
            
            for nt = 1:9
                tem = [0;0;0];
                tem(seq(nt)) = -hG(seq(nt))/sum(seq==seq(nt));
                del_q(ths(nt)) = del_q(ths(nt)) + pinv(AG(:,ths(nt)))*([tem;0;0;0]);
                theta(ths(nt)) = theta(ths(nt)) + pinv(AG(:,ths(nt)))*([tem;0;0;0]);
                sdel_q(ths(nt)) = sdel_q(ths(nt)) + del_q(ths(nt));
            end
            [com,cm,P,L] = ForwKin(theta,Tr_mat,del_q);
            
        end
    end
    if(flag==0)
        disp('Solution Does not Exist');
        q = [];
        Dq = [];
        pos =[];
        HG = [];
        break;
    end
end
% s_d(cm,com)
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
%         fprintf('Enter = %d\n',ang);
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
%             figure(1);
%             hold on;
%             plot([f2_z(1),f2_z(2),f1_z(3),f1_z(4),f2_z(1)],[f2_y(1),f2_y(2),f1_y(3),f1_y(4),f2_y(1)]);
%             plot(pz,py,'ko');
%             plot(com.z,com.y,'ro');
%             hold off;
            %disp(com);
            stable = inpolygon(pz,py,[f2_z(1),f2_z(2),f1_z(3),f1_z(4),f2_z(1)],[f2_y(1),f2_y(2),f1_y(3),f1_y(4),f2_y(1)]);
        end
        
    end
end


