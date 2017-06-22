%#####################################################################################################
%# Function : Calculates the forward kinematics of the robote#                                                          #   
%#                                                                                                                      #       
%# Input(s)  : theta, T_mat(transformation matrix on base), dq(angular velocity of each joint)                                                                                #
%#                                                                                                                      #                
%# Ouptut(s) :com_final(COM of the robot), cm(COM of each part), P_final(linear momentum of the robot),
%#            L_final(angular momentum of the robot),P(linear momentum of each part),L(angular momentum of each part)   #                   # 
%#                                                                                                   #       
%# Example: ForwKin(theta,T_mat,dq)                                                                #   
%#                                                                                                   #       
%#                                                                                                   #               
%#                                                                                                   #                                       
%#####################################################################################################

function [com_final,cm,P_final,L_final,P,L] = ForwKin(theta,T_mat,dq)
global robot
global base
global move_base
load model_data_new
load com_data_latest
%Robot_data

for i=1:25
    com.x(i) = sum(cm_data(i).x.*cm_data(i).mass)/sum(cm_data(i).mass);
    com.y(i) = sum(cm_data(i).y.*cm_data(i).mass)/sum(cm_data(i).mass);
    com.z(i) = sum(cm_data(i).z.*cm_data(i).mass)/sum(cm_data(i).mass);
    com.mass(i) = sum(cm_data(i).mass);
    
    if(i==7)
        xf = sum(cm_data(i).x_fixed.*cm_data(i).mass_fixed);
        yf = sum(cm_data(i).y_fixed.*cm_data(i).mass_fixed);
        zf = sum(cm_data(i).z_fixed.*cm_data(i).mass_fixed);
        mf = sum(cm_data(i).mass_fixed);
       
    end
    com.I(:,:,i) = cm_data(i).I;
end

if(base == 'rleg') %(only this base is changed)
    theta(16) = theta(11);
    %theta(21) = theta(7); %(Equal if the bot is not in double spport phase)
    
    th1(2:16) = th1(2:16) + theta(1:15);
    th2 = th2 + theta(16:20);
    th3 = th3 + theta(21:27);
    theta_t = [th1,th2,th3];
    
    
    if(move_base == 1)
        Tr_mat = T_mat;
    else
        Tr_mat = eye(4);
    end
    
    % For 0 - 17
    T(:,:,1) = Hom_Trans(alp1(1),l0,0,th1(1)); %T_w_0
    T(:,:,2) = Tr_mat*Hom_Trans(alp1(2),l1,0,th1(2)); %T_0_1
    T(:,:,3) = Hom_Trans(alp1(3),l2,0,th1(3)); %T_1_2
    T(:,:,4) = Hom_Trans(alp1(4),l3,-d3,th1(4)); %T_2_3
    T(:,:,5) = Hom_Trans(alp1(5),d4,0,th1(5)); %T_3_4
    T(:,:,6) = Hom_Trans(alp1(6),l5,l4,th1(6)); %T_4_5
    T(:,:,7) = Hom_Trans(alp1(7),l6,d6,th1(7)); %T_5_6
    
    T(:,:,8) = Hom_Trans(alp1(8),0,d5,th1(8)); %T_6_7
    
    
    T(:,:,9) = Hom_Trans(alp1(9),0,-d8,th1(9)); %T_7_8
    T(:,:,10) = Hom_Trans(alp1(10),-d9,l8+l9,th1(10)); %T_8_9
    T(:,:,11) = Hom_Trans(alp1(11),0,0,th1(11)); %T_9_10
    
    %For 13 - 17
    T(:,:,12) = Hom_Trans(alp1(12),l11,0,th1(12)); %T_10_13
    T(:,:,13) = Hom_Trans(alp1(13),0,-(l13+d14),th1(13)); %T_13_14
    T(:,:,14) = Hom_Trans(alp1(14),0,0,th1(14)); %T_14_15
    T(:,:,15) = Hom_Trans(alp1(15),d5,-(l15+l16),th1(15)); %T_15_16
    T(:,:,16) = Hom_Trans(alp1(16),-l17,0,th1(16)); %T_16_17
    
    %For 13' - 17'
    T1(:,:,1) = Hom_Trans(alp2(1),l11,0,th2(1)); %T_10_13'
    T1(:,:,2) = Hom_Trans(alp2(2),0,(l13+d14),th2(2)); %T_13'_14'
    T1(:,:,3) = Hom_Trans(alp2(3),0,0,th2(3)); %T_14'_15'
    T1(:,:,4) = Hom_Trans(alp2(4),d5,-(l15+l16),th2(4)); %T_15'_16'
    T1(:,:,5) = Hom_Trans(alp2(5),-l17,0,th2(5)); %T_16'_17'
    
    %For 5' - 00'
    T2(:,:,1) = Hom_Trans(alp3(1),l6,2*d5,th3(1)); %T_5_6
    T2(:,:,2) = Hom_Trans(alp3(2),l5,-d6,th3(2)); %T_4_5
    T2(:,:,3) = Hom_Trans(alp3(3),d4,-l4,th3(3)); %T_3_4
    T2(:,:,4) = Hom_Trans(alp3(4),l3,-d3,th3(4)); %T_2_3
    T2(:,:,5) = Hom_Trans(alp3(5),l2,0,th3(5)); %T_1_2
    T2(:,:,6) = Hom_Trans(alp3(6),l1,0,th3(6)); %T_0_1
    T2(:,:,7) = Hom_Trans(alp3(7),l0,0,th3(7)); %T_w_0
    for i=1:15
        if(i==1)
            T_m(:,:,i) = T(:,:,i+1);
            temp = Hom_Trans(alp1(2),l1,0,th1(2));
            R_loc(:,:,i+1) = temp(1:3,1:3);
            b_t(:,i+1) = temp*[0;0;0;1];
            br_t(:,i+1) = inv(temp)*[0;0;0;1];
            a_t(:,i+1) = temp(1:3,1:3)*[0;0;1];
            ar_t(:,i+1) = temp(1:3,1:3)'*[0;0;1];
        else
            T_m(:,:,i) = T_m(:,:,i-1)*T(:,:,i+1);
            b_t(:,i+1) = T(:,:,i+1)*[0;0;0;1];
            br_t(:,i+1) = inv(T(:,:,i+1))*[0;0;0;1];
            a_t(:,i+1) = T(1:3,1:3,i+1)*[0;0;1];
            ar_t(:,i+1) = T(1:3,1:3,i+1)'*[0;0;1];
            R_loc(:,:,i+1) = T(1:3,1:3,i+1);
        end
        
        R(:,:,i) = T_m(1:3,1:3,i);
        R_t(:,:,i+1) = R(:,:,i);
    end
    
    T_ra = T_m(:,:,15);
    
    for i=1:6
        if(i==1)
            T1_m(:,:,i) = T_m(:,:,10);
            R1(:,:,i) = T1_m(1:3,1:3,i);
        else
            T1_m(:,:,i) = T1_m(:,:,i-1)*T1(:,:,i-1);
            R1(:,:,i) = T1_m(1:3,1:3,i);
            R_t(:,:,i+15) = R1(:,:,i);
            b_t(:,i+15) = T1(:,:,i-1)*[0;0;0;1];
            br_t(:,i+15) = inv(T1(:,:,i-1))*[0;0;0;1];
            a_t(:,i+15) = T1(1:3,1:3,i-1)*[0;0;1];
            ar_t(:,i+15) = T1(1:3,1:3,i-1)'*[0;0;1];
            R_loc(:,:,i+15) = T1(1:3,1:3,i-1);
        end
    end
    
    T_la = T1_m(:,:,6);
    
    for i=1:8
        if(i==1)
            T2_m(:,:,i) = T_m(:,:,6);
            R2(:,:,i) = T2_m(1:3,1:3,i);
        else
            T2_m(:,:,i) = T2_m(:,:,i-1)*T2(:,:,i-1);
            R2(:,:,i) = T2_m(1:3,1:3,i);
            R_t(:,:,i+20) = R2(:,:,i);
            b_t(:,i+20) = T2(:,:,i-1)*[0;0;0;1];
            br_t(:,i+20) = inv(T2(:,:,i-1))*[0;0;0;1];
            a_t(:,i+20) = T2(1:3,1:3,i-1)*[0;0;1];
            ar_t(:,i+20) = T2(1:3,1:3,i-1)'*[0;0;1];
            R_loc(:,:,i+20) = T2(1:3,1:3,i-1);
        end
    end
    
    T_lg = T2_m(:,:,8);
    
    % %Calculating Positions
    pts(:,1) = Tr_mat*[0;0;0;1];
    b_t(:,1) = [0;0;0;1];
    br_t(:,1) = [0;0;0;1];
    
    R_t(:,:,1) = Tr_mat(1:3,1:3,:);
    R_loc(:,:,1) = Tr_mat(1:3,1:3);
    draw_pts(:,1) = Tr_mat*(T(:,:,1)^-1)*[0;0;0;1];
    draw_pts(:,2) = Tr_mat*[0;0;0;1];
    
    u(:,1) = R_t(:,:,1)*[0;0;1];
    a_t(:,1) = [0;0;1];
    ar_t(:,1) = [0;0;1];
    for i=1:15
        pts(:,i+1) = T_m(:,:,i)*[0;0;0;1];
        draw_pts(:,i+2) = T_m(:,:,i)*[0;0;0;1];
        u(:,i+1) = R(:,:,i)*[0;0;1];
    end
    
    for i=1:6
        pts1(:,i) = T1_m(:,:,i)*[0;0;0;1];
        u1(:,i) = R1(:,:,i)*[0;0;1];
    end
    
    for i=1:8
        pts2(:,i) = T2_m(:,:,i)*[0;0;0;1];
        u2(:,i) = R2(:,:,i)*[0;0;1];
    end
    p = [pts, pts1(:,2:6), pts2(:,2:8)];
    u = [u, u1(:,2:6), u2(:,2:8)];
    
    draw_pts = [draw_pts, pts1(:,2:6), pts2(:,2:8)];
    draw_pts(:,6) = T_m(:,:,4)*[0;0;l4;1];
    draw_pts(:,8) = T_m(:,:,6)*[0;0;d5;1];
    draw_pts(:,10) = T_m(:,:,8)*[0;0;l8;1];
    draw_pts(:,23) = T2_m(:,:,2)*[0;0;-d6;1];
    draw_pts(:,13) = T_m(:,:,11)*[0;0;-(l13);1];
    draw_pts(:,18) = T1_m(:,:,2)*[0;0;(l13);1];
    
    for i=1:28
        robot.parts(i).axis_loc = p(1:3,i);
        robot.parts(i).joint_loc = draw_pts(1:3,i+1);
        robot.parts(i).Z_w = u(1:3,i);
        robot.parts(i).R_mat = R_t(:,:,i);
        robot.parts(i).Xs = [R_loc(:,:,i)'                   zeros(3,3);
                             R_loc(:,:,i)'*hat(b_t(1:3,i))'  R_loc(:,:,i)'];
        robot.parts(i).b = b_t(1:3,i);
        robot.parts(i).br = br_t(1:3,i);
        robot.parts(i).a = a_t(1:3,i);
        robot.parts(i).ar = ar_t(1:3,i);
    end
    
    
    T = [0 0 -1 0;0 1 0 0; 1 0 0 l0;0 0 0 1];
    T=eye(4);
    for i=1:29
        draw_pts(:,i) = T*draw_pts(:,i);
    end
    robot.draw = draw_pts(1:3,:);
    
    idx = 1:25;
    
    cm.mass = com.mass(idx);
    cm.I(:,:,:) = com.I(:,:,idx);
    
    x = com.x(idx);
    y = com.y(idx);
    z = com.z(idx);
    
    It = cm.mass(1)*([x(1);y(1);z(1)]'*[x(1);y(1);z(1)] - [x(1);y(1);z(1)]*[x(1);y(1);z(1)]'); % Translation to local frame
    I_loc(:,:,1) = It + cm.I(:,:,1);
    
    for j=1:25
        if (j==1 || j==8 || j==11)
            %pt(:,j) = [y(j);-x(j);z(j);1];
            tmp = Hom_Trans(0,0,0,-90);
            pt(:,j) = tmp*[x(j);y(j);z(j);1];%1
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = tmp(1:3,1:3)*I_loc(:,:,j)*tmp(1:3,1:3)';
            
        elseif(j==5)
            
            %pt(:,j) = [-z(j);-x(j);y(j);1];%5
            tmp = Hom_Trans(90,0,0,-90);
            pt(:,j) = tmp*[x(j);y(j);z(j);1];%5
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = tmp(1:3,1:3)*I_loc(:,:,j)*tmp(1:3,1:3)';
            
            
        elseif(j==6)
            
            pt(:,j) = [x(j);y(j);z(j);1]; %6,10
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);                                                            %Rotation not needed here
            
            
        elseif(j==13 || j==20 || j==17 || j==25)
            
            tmp = Hom_Trans(0,0,0,90);
            pt(:,j) = tmp*[x(j);y(j);z(j);1];
            %pt(:,j) = [-y(j);x(j);z(j);1];%13,17,25
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = tmp(1:3,1:3)*I_loc(:,:,j)*tmp(1:3,1:3)';
            
        elseif(j==9 || j==14 || j==18 || j==21)
            
            tmp = Hom_Trans(90,0,0,0);
            pt(:,j) = tmp*[x(j);y(j);z(j);1];
            %pt(:,j) = [x(j);-z(j);y(j);1];%14,18,21
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = tmp(1:3,1:3)*I_loc(:,:,j)*tmp(1:3,1:3)';
            
        elseif(j==10 || j==15 || j==19 || j==22)
            
            tmp = Hom_Trans(0,0,0,0);
            tmp(1:3,1:3) = [0 0 1;0 1 0;-1 0 0]*[-1 0 0;0 -1 0; 0 0 1];
            pt(:,j) = tmp*[x(j);y(j);z(j);1];
            %pt(:,j) = [z(j);-y(j);x(j);1];%15,19,22
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = tmp(1:3,1:3)*I_loc(:,:,j)*tmp(1:3,1:3)';
            
            
        elseif(j==23 || j==24)
            
            tmp = Hom_Trans(0,0,0,0);
            tmp(1:3,1:3) = [1 0 0;0 0 -1;0 1 0]*[0 -1 0;1 0 0; 0 0 1];
            pt(:,j) = tmp*[x(j);y(j);z(j);1];
            %pt(:,j) = [-y(j);-z(j);x(j);1];%23,24
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = tmp(1:3,1:3)*I_loc(:,:,j)*tmp(1:3,1:3)';
            
        else
            
            tmp = Hom_Trans(0,0,0,0);
            tmp(1:3,1:3) = [1 0 0;0 0 1;0 -1 0]*[0 1 0;-1 0 0; 0 0 1];
            pt(:,j) = tmp*[x(j);y(j);z(j);1];
            if(j==7)
                comf = [xf;yf;zf]/mf;
                tmp2 = tmp*[comf;1];
                xf = tmp2(1);
                yf = tmp2(2);
                zf = tmp2(3);
            end
            %pt(:,j) = [y(j);z(j);x(j);1];  % 2,3,4,7,12,16
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = tmp(1:3,1:3)*I_loc(:,:,j)*tmp(1:3,1:3)';
            
        end
    end
    
    tmp = Tr_mat*[pt(1,1);pt(2,1);pt(3,1);1];
    
    cm.x(1) = tmp(1);
    cm.y(1) = tmp(2);
    cm.z(1) = tmp(3);

    [v,w] = standing_vel(dq);
    c1 = v(:,1)+ cross(w(:,1),robot.parts(1).R_mat*[pt(1,1);pt(2,1);pt(3,1)]);
    P(:,1) = cm.mass(1)*c1;                                                                 %Linear momentum calculation
    
    L(:,1) = cross([cm.x(1);cm.y(1);cm.z(1)],P(:,1)) + robot.parts(1).R_mat*I_loc(:,:,1)*robot.parts(1).R_mat'*w(:,1);    %Angular momentum
    
    j = 2;
    it=2;
    
    for i=1:14
        if(j==5)
            pt(3,j) = pt(3,j) + l4;
        elseif(j==9)
            pt(3,j) = pt(3,j) + l8;
        elseif(j==12)
            pt(3,j) = pt(3,j) - l13;
        end
        
        if(j>=7)
            pt(1:3,j) = rotVecAroundArbAxis([pt(1,j),pt(2,j),pt(3,j)],robot.parts(it).Z_w',theta(it));
        end
        
        if(j==7)

            pt(1:3,j) = (pt(1:3,j)*cm.mass(j) + [xf;yf;zf]*mf)/(cm.mass(j)+mf);
            pt(3,j) = pt(3,j) + d5;
            
        end
        
        tmp = T_m(:,:,i)*pt(:,j);
        
        cm.x(j) = tmp(1);
        cm.y(j) = tmp(2);
        cm.z(j) = tmp(3);
        
        c1 = v(:,it)+ cross(w(:,it),robot.parts(it).R_mat*[pt(1,j);pt(2,j);pt(3,j)]);
        P(:,j) = cm.mass(j)*c1;                                                                 %Linear momentum calculation
        
        L(:,j) = cross([cm.x(j);cm.y(j);cm.z(j)],P(:,j)) + robot.parts(it).R_mat*I_loc(:,:,j)*robot.parts(it).R_mat'*w(:,it);    %Angular momentum
        
        j = j+1;
        it = it+1;
    end
    
    it = it+1;
    
    for i=2:5
        if(j==16)
            pt(3,j) = pt(3,j) + l13;
        end
        
        pt(1:3,j) = rotVecAroundArbAxis([pt(1,j),pt(2,j),pt(3,j)],robot.parts(it).Z_w',theta(it));
        tmp = T1_m(:,:,i)*pt(:,j);
        
        cm.x(j) = tmp(1);
        cm.y(j) = tmp(2);
        cm.z(j) = tmp(3);
        
        c1 = v(:,it)+ cross(w(:,it),robot.parts(it).R_mat*[pt(1,j);pt(2,j);pt(3,j)]);
        P(:,j) = cm.mass(j)*c1;                                                                 %Linear momentum calculation
        
        L(:,j) = cross([cm.x(j);cm.y(j);cm.z(j)],P(:,j)) + robot.parts(it).R_mat*I_loc(:,:,j)*robot.parts(it).R_mat'*w(:,it);    %Angular momentum
        
        j = j+1;
        it = it+1;
    end
    
    it = it+1;
    
    for i=2:7
        if(j==20)
            pt(3,j) = pt(3,j) - d6;
        end
        
        pt(1:3,j) = rotVecAroundArbAxis([pt(1,j),pt(2,j),pt(3,j)],robot.parts(it).Z_w',theta(it));
        tmp = T2_m(:,:,i)*pt(:,j);
        
        cm.x(j) = tmp(1);
        cm.y(j) = tmp(2);
        cm.z(j) = tmp(3);
        
        c1 = v(:,it)+ cross(w(:,it),robot.parts(it).R_mat*[pt(1,j);pt(2,j);pt(3,j)]);
        P(:,j) = cm.mass(j)*c1;                                                                 %Linear momentum calculation
        
        L(:,j) = cross([cm.x(j);cm.y(j);cm.z(j)],P(:,j)) + robot.parts(it).R_mat*I_loc(:,:,j)*robot.parts(it).R_mat'*w(:,it);    %Angular momentum
        
        j = j+1;
        it = it+1;
        
    end
    
    
    com_final.x = sum(cm.x.*cm.mass)/sum(cm.mass);
    com_final.y = sum(cm.y.*cm.mass)/sum(cm.mass);
    com_final.z = sum(cm.z.*cm.mass)/sum(cm.mass);
    com_final.mass = sum(cm.mass);
    P_final = sum(P,2);
    L_final = sum(L,2);
    
    cm.x(26)=0;cm.y(26)=0;cm.z(26)=0;
    I_loc(:,:,26) = eye(3);
    cm.mass(26) = 0;
    P(:,26) = [0;0;0];
    L(:,26) = [0;0;0];
    pt(:,26) = [0;0;0;1];
    
    
    j=1;
    
    for i = 1:28
        robot.parts(i).mass = cm.mass(j);
        robot.parts(i).I = I_loc(:,:,j);
        robot.parts(i).Is = Is(cm.mass(j),[pt(1,j),pt(2,j),pt(3,j)],I_loc(:,:,j));
        robot.parts(i).com_g =[cm.x(j),cm.y(j),cm.z(j)];
        robot.parts(i).com_l =[pt(1,j),pt(2,j),pt(3,j)];
        robot.parts(i).P = P(:,j);
        robot.parts(i).L = L(:,j);
        robot.parts(i).Xpg = [robot.parts(i).R_mat'                                                                         zeros(3,3);
                              robot.parts(i).R_mat'*hat(robot.parts(i).joint_loc - [com_final.x;com_final.y;com_final.z])'  robot.parts(i).R_mat'];
        if(i==16 || i==21 || i==28)
            robot.parts(i).mass = 0;
            robot.parts(i).I = zeros(3,3);
            robot.parts(i).Is = zeros(6,6);
            robot.parts(i).P = [0;0;0];
            robot.parts(i).L = [0;0;0];
            robot.parts(i).com_g =[0,0,0];
            robot.parts(i).com_l =[0,0,0];
        else
            j=j+1;
        end
    end
    
    
end

if(base == 'lleg')
    
    theta(16) = theta(11); %since 16 is redundant
    theta(21) = theta(7);  %since 21 is redundant
    
    %Changes in the model
    th1(7) = 114.0254;
    th1(8) = -th1(8);
    th3(2) = 114.0254;
    th3(1) = -th3(1);
    
    th1(2:16) = th1(2:16) + theta(1:15);
    th2 = th2 + theta(16:20);
    th3 = th3 + theta(21:27);
    theta_t = [th1,th2,th3];
    
    
    if(move_base == 1)
        Tr_mat = T_mat;
    else
        Tr_mat = eye(4);
    end
    
    % For 0 - 17
    T(:,:,1) = Hom_Trans(alp1(1),l0,0,th1(1)); %T_w_0
    T(:,:,2) = Tr_mat*Hom_Trans(alp1(2),l1,0,th1(2)); %T_0_1
    T(:,:,3) = Hom_Trans(alp1(3),l2,0,th1(3)); %T_1_2
    T(:,:,4) = Hom_Trans(alp1(4),l3,d3,th1(4)); %T_2_3
    T(:,:,5) = Hom_Trans(alp1(5),d4,0,th1(5)); %T_3_4
    T(:,:,6) = Hom_Trans(alp1(6),-l5,l4,th1(6)); %T_4_5
    T(:,:,7) = Hom_Trans(alp1(7),sqrt(l6^2 + d5^2),d6,th1(7)); %T_5_6
    
    T(:,:,8) = Hom_Trans(alp1(8),0,0,th1(8)); %T_6_7
    
    
    T(:,:,9) = Hom_Trans(alp1(9),-d8,0,th1(9)); %T_7_8
    T(:,:,10) = Hom_Trans(alp1(10),0,l8+l9,th1(10)); %T_8_9
    T(:,:,11) = Hom_Trans(alp1(11),0,-d9,th1(11)); %T_9_10
    
    %For 13 - 17
    T(:,:,12) = Hom_Trans(alp1(12),l11,l13,th1(12)); %T_10_13
    T(:,:,13) = Hom_Trans(alp1(13),0,d14,th1(13)); %T_13_14
    T(:,:,14) = Hom_Trans(alp1(14),0,0,th1(14)); %T_14_15
    T(:,:,15) = Hom_Trans(alp1(15),d5,-(l15+l16),th1(15)); %T_15_16
    T(:,:,16) = Hom_Trans(alp1(16),-l17,0,th1(16)); %T_16_17
    
    %For 13' - 17'
    T1(:,:,1) = Hom_Trans(alp2(1),l11,-l13,th2(1)); %T_10_13'
    T1(:,:,2) = Hom_Trans(alp2(2),0,-d14,th2(2)); %T_13'_14'
    T1(:,:,3) = Hom_Trans(alp2(3),0,0,th2(3)); %T_14'_15'
    T1(:,:,4) = Hom_Trans(alp2(4),d5,-(l15+l16),th2(4)); %T_15'_16'
    T1(:,:,5) = Hom_Trans(alp2(5),-l17,0,th2(5)); %T_16'_17'
    
    %For 5' - 00'
    T2(:,:,1) = Hom_Trans(alp3(1),sqrt(l6^2 + d5^2),-d6,th3(1)); %T_5_6
    T2(:,:,2) = Hom_Trans(alp3(2),-l5,0,th3(2)); %T_4_5
    T2(:,:,3) = Hom_Trans(alp3(3),d4,-l4,th3(3)); %T_3_4
    T2(:,:,4) = Hom_Trans(alp3(4),l3,d3,th3(4)); %T_2_3
    T2(:,:,5) = Hom_Trans(alp3(5),l2,0,th3(5)); %T_1_2
    T2(:,:,6) = Hom_Trans(alp3(6),l1,0,th3(6)); %T_0_1
    T2(:,:,7) = Hom_Trans(alp3(7),l0,0,th3(7)); %T_w_0
    for i=1:15
        if(i==1)
            T_m(:,:,i) = T(:,:,i+1);
            temp = Hom_Trans(alp1(2),l1,0,th1(2));
            b_t(:,i+1) = temp*[0;0;0;1];
            br_t(:,i+1) = inv(temp)*[0;0;0;1];
            a_t(:,i+1) = temp(1:3,1:3)*[0;0;1];
            ar_t(:,i+1) = temp(1:3,1:3)'*[0;0;1];
        else
            T_m(:,:,i) = T_m(:,:,i-1)*T(:,:,i+1);
            b_t(:,i+1) = T(:,:,i+1)*[0;0;0;1];
            br_t(:,i+1) = inv(T(:,:,i+1))*[0;0;0;1];
            a_t(:,i+1) = T(1:3,1:3,i+1)*[0;0;1];
            ar_t(:,i+1) = T(1:3,1:3,i+1)'*[0;0;1];
        end
        
        R(:,:,i) = T_m(1:3,1:3,i);
        R_t(:,:,i+1) = R(:,:,i);
    end
    
    T_ra = T_m(:,:,15);
    
    for i=1:6
        if(i==1)
            T1_m(:,:,i) = T_m(:,:,10);
            R1(:,:,i) = T1_m(1:3,1:3,i);
        else
            T1_m(:,:,i) = T1_m(:,:,i-1)*T1(:,:,i-1);
            R1(:,:,i) = T1_m(1:3,1:3,i);
            R_t(:,:,i+16) = R1(:,:,i);
            b_t(:,i+16) = T1(:,:,i-1)*[0;0;0;1];
            br_t(:,i+16) = inv(T1(:,:,i-1))*[0;0;0;1];
            a_t(:,i+16) = T1(1:3,1:3,i-1)*[0;0;1];
            ar_t(:,i+16) = T1(1:3,1:3,i-1)'*[0;0;1];
        end
    end
    
    T_la = T1_m(:,:,6);
    
    for i=1:8
        if(i==1)
            T2_m(:,:,i) = T_m(:,:,6);
            R2(:,:,i) = T2_m(1:3,1:3,i);
        else
            T2_m(:,:,i) = T2_m(:,:,i-1)*T2(:,:,i-1);
            R2(:,:,i) = T2_m(1:3,1:3,i);
            R_t(:,:,i+20) = R2(:,:,i);
            b_t(:,i+20) = T2(:,:,i-1)*[0;0;0;1];
            br_t(:,i+20) = inv(T2(:,:,i-1))*[0;0;0;1];
            a_t(:,i+20) = T2(1:3,1:3,i-1)*[0;0;1];
            ar_t(:,i+20) = T2(1:3,1:3,i-1)'*[0;0;1];
        end
    end
    
    T_lg = T2_m(:,:,8);
    
    % %Calculating Positions
    pts(:,1) = Tr_mat*[0;0;0;1];
    b_t(:,1) = [0;0;0;1];
    br_t(:,1) = [0;0;0;1];
    
    R_t(:,:,1) = Tr_mat(1:3,1:3,:);
    draw_pts(:,1) = Tr_mat*(T(:,:,1)^-1)*[0;0;0;1];
    draw_pts(:,2) = Tr_mat*[0;0;0;1];
    
    u(:,1) = R_t(:,:,1)*[0;0;1];
    a_t(:,1) = [0;0;1];
    ar_t(:,1) = [0;0;1];
    
    for i=1:15
        pts(:,i+1) = T_m(:,:,i)*[0;0;0;1];
        draw_pts(:,i+2) = T_m(:,:,i)*[0;0;0;1];
        u(:,i+1) = R(:,:,i)*[0;0;1];
    end
    
    for i=1:6
        pts1(:,i) = T1_m(:,:,i)*[0;0;0;1];
        u1(:,i) = R1(:,:,i)*[0;0;1];
    end
    
    for i=1:8
        pts2(:,i) = T2_m(:,:,i)*[0;0;0;1];
        u2(:,i) = R2(:,:,i)*[0;0;1];
    end
    p = [pts, pts1(:,2:6), pts2(:,2:8)];
    u = [u, u1(:,2:6), u2(:,2:8)];
    
    draw_pts = [draw_pts, pts1(:,2:6), pts2(:,2:8)];
    draw_pts(:,6) = T_m(:,:,4)*[0;0;l4;1];
    draw_pts(:,10) = T_m(:,:,8)*[0;0;l8;1];
    draw_pts(:,11) = T_m(:,:,9)*[0;0;-d9;1];
    
    for i=1:28
        robot.parts(i).axis_loc = p(1:3,i);
        robot.parts(i).joint_loc = draw_pts(1:3,i+1);
        robot.parts(i).Z_w = u(1:3,i);
        robot.parts(i).R_mat = R_t(:,:,i);
        robot.parts(i).b = b_t(1:3,i);
        robot.parts(i).br = br_t(1:3,i);
        robot.parts(i).a = a_t(1:3,i);
        robot.parts(i).ar = ar_t(1:3,i);
    end
    
    
    T = [0 0 1 0;0 1 0 0; 1 0 0 l0;0 0 0 1];
%     T=eye(4);
    for i=1:29
        draw_pts(:,i) = T*draw_pts(:,i);
    end
    robot.draw = draw_pts(1:3,:);
    
    idx = [1 2 4 3 13 11 9 14 17 15 16 18 20 19 21 22 24 23 25 10 12 5 6 8 7];
    
    cm.mass = com.mass(idx);
    cm.I(:,:,:) = com.I(:,:,idx);
    
    x = com.x(idx);
    y = com.y(idx);
    z = com.z(idx);
    
    It = cm.mass(1)*([x(1);y(1);z(1)]'*[x(1);y(1);z(1)] - [x(1);y(1);z(1)]*[x(1);y(1);z(1)]'); % Translation to local frame
    I_loc(:,:,1) = It + cm.I(:,:,1);
    
    for j=1:25
        if (j==1)
            
            %pt(:,j) = [y(j);-x(j);z(j);1];
            tmp = Hom_Trans(0,0,0,-90);
            pt(:,j) = tmp*[x(j);y(j);z(j);1];%1
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = tmp(1:3,1:3)*I_loc(:,:,j)*tmp(1:3,1:3)';
            
        elseif(j==5)
            
            %pt(:,j) = [-z(j);-x(j);y(j);1];%5
            tmp = Hom_Trans(90,0,0,-90);
            pt(:,j) = tmp*[x(j);y(j);z(j);1];%5
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = tmp(1:3,1:3)*I_loc(:,:,j)*tmp(1:3,1:3)';
            
            
        elseif(j==6 || j==10)
            
            pt(:,j) = [x(j);y(j);z(j);1]; %6,10
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);                                                            %Rotation not needed here
            
        elseif(j==7)
            
            p = [x(j);y(j);z(j)];
            R = [cosd(-114.0254) -sind(-114.0254) 0;
                sind(-114.0254)  cosd(-114.0254)  0;
                0               0             1;]; %7
            %R = eye(3);
            pt(:,j) = [R*p;1];
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = R*I_loc(:,:,j)*R';
            
        elseif(j==9)
            
            %pt(:,j) = [z(j);x(j);y(j);1];%9
            tmp = Hom_Trans(90,0,0,90);
            pt(:,j) = tmp*[x(j);y(j);z(j);1];%9
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = tmp(1:3,1:3)*I_loc(:,:,j)*tmp(1:3,1:3)';
            
        elseif(j==13 || j==17 || j==25)
            
            tmp = Hom_Trans(0,0,0,90);
            pt(:,j) = tmp*[x(j);y(j);z(j);1];
            %pt(:,j) = [-y(j);x(j);z(j);1];%13,17,25
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = tmp(1:3,1:3)*I_loc(:,:,j)*tmp(1:3,1:3)';
            
        elseif(j==14 || j==18 || j==21)
            
            tmp = Hom_Trans(90,0,0,0);
            pt(:,j) = tmp*[x(j);y(j);z(j);1];
            %pt(:,j) = [x(j);-z(j);y(j);1];%14,18,21
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = tmp(1:3,1:3)*I_loc(:,:,j)*tmp(1:3,1:3)';
            
        elseif(j==15 || j==19 || j==22)
            
            tmp = Hom_Trans(0,0,0,0);
            tmp(1:3,1:3) = [0 0 1;0 1 0;-1 0 0]*[-1 0 0;0 -1 0; 0 0 1];
            pt(:,j) = tmp*[x(j);y(j);z(j);1];
            %pt(:,j) = [z(j);-y(j);x(j);1];%15,19,22
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = tmp(1:3,1:3)*I_loc(:,:,j)*tmp(1:3,1:3)';
            
        elseif(j==20)
            
            p = [x(j);y(j);z(j)];
            R = [cosd(114.0254) -sind(114.0254) 0;
                sind(114.0254)  cosd(114.0254)  0;
                0               0             1;];%20
            pt(:,j) = [R*p;1];
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = R*I_loc(:,:,j)*R';
            
        elseif(j==23 || j==24)
            
            tmp = Hom_Trans(0,0,0,0);
            tmp(1:3,1:3) = [1 0 0;0 0 -1;0 1 0]*[0 -1 0;1 0 0; 0 0 1];
            pt(:,j) = tmp*[x(j);y(j);z(j);1];
            %pt(:,j) = [-y(j);-z(j);x(j);1];%23,24
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = tmp(1:3,1:3)*I_loc(:,:,j)*tmp(1:3,1:3)';
            
        else
            
            tmp = Hom_Trans(0,0,0,0);
            tmp(1:3,1:3) = [1 0 0;0 0 1;0 -1 0]*[0 1 0;-1 0 0; 0 0 1];
            pt(:,j) = tmp*[x(j);y(j);z(j);1];
            %pt(:,j) = [y(j);z(j);x(j);1];  % 2,3,4,8,11,12,16
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = tmp(1:3,1:3)*I_loc(:,:,j)*tmp(1:3,1:3)';
            
        end
    end
    
    pt(1:3,1) = rotVecAroundArbAxis([pt(1,1),pt(2,1),pt(3,1)],robot.parts(1).Z_w',theta(1));
    tmp = Tr_mat*[pt(1,1);pt(2,1);pt(3,1);1];
    
    cm.x(1) = tmp(1);
    cm.y(1) = tmp(2);
    cm.z(1) = tmp(3);
    
    %     for i = 1:28
    %         robot.parts(i).v =[0;0;0];
    %         robot.parts(i).w =[0;0;0];
    %     end
    
    [v,w] = standing_vel(dq);
    
    c1 = v(:,1)+ cross(w(:,1),robot.parts(1).R_mat*[pt(1,1);pt(2,1);pt(3,1)]);
    P(:,1) = cm.mass(1)*c1;                                                                 %Linear momentum calculation
    
    L(:,1) = cross([cm.x(1);cm.y(1);cm.z(1)],P(:,1)) + robot.parts(1).R_mat*I_loc(:,:,1)*robot.parts(1).R_mat'*w(:,1);    %Angular momentum
    
    j = 2;
    it=2;
    
    for i=1:14
        
        pt(1:3,j) = rotVecAroundArbAxis([pt(1,j),pt(2,j),pt(3,j)],robot.parts(it).Z_w',theta(it));
        tmp = T_m(:,:,i)*pt(:,j);
        
        cm.x(j) = tmp(1);
        if (i==4)
            cm.x(j) = tmp(1)+l4;
        else
            cm.x(j) = tmp(1);
        end
        cm.y(j) = tmp(2);
        cm.z(j) = tmp(3);
        
        c1 = v(:,it)+ cross(w(:,it),robot.parts(it).R_mat*[pt(1,j);pt(2,j);pt(3,j)]);
        P(:,j) = cm.mass(j)*c1;                                                                 %Linear momentum calculation
        
        L(:,j) = cross([cm.x(j);cm.y(j);cm.z(j)],P(:,j)) + robot.parts(it).R_mat*I_loc(:,:,j)*robot.parts(it).R_mat'*w(:,it);    %Angular momentum
        
        
        j = j+1;
        it = it+1;
    end
    
    it = it+1;
    
    for i=2:5
        
        pt(1:3,j) = rotVecAroundArbAxis([pt(1,j),pt(2,j),pt(3,j)],robot.parts(it).Z_w',theta(it));
        tmp = T1_m(:,:,i)*pt(:,j);
        
        cm.x(j) = tmp(1);
        cm.y(j) = tmp(2);
        cm.z(j) = tmp(3);
        
        c1 = v(:,it)+ cross(w(:,it),robot.parts(it).R_mat*[pt(1,j);pt(2,j);pt(3,j)]);
        P(:,j) = cm.mass(j)*c1;                                                                 %Linear momentum calculation
        
        L(:,j) = cross([cm.x(j);cm.y(j);cm.z(j)],P(:,j)) + robot.parts(it).R_mat*I_loc(:,:,j)*robot.parts(it).R_mat'*w(:,it);    %Angular momentum
        
        j = j+1;
        it = it+1;
    end
    
    it = it+1;
    
    for i=2:7
        
        pt(1:3,j) = rotVecAroundArbAxis([pt(1,j),pt(2,j),pt(3,j)],robot.parts(it).Z_w',theta(it));
        tmp = T2_m(:,:,i)*pt(:,j);
        
        cm.x(j) = tmp(1);
        cm.y(j) = tmp(2);
        cm.z(j) = tmp(3);
        
        c1 = v(:,it)+ cross(w(:,it),robot.parts(it).R_mat*[pt(1,j);pt(2,j);pt(3,j)]);
        P(:,j) = cm.mass(j)*c1;                                                                 %Linear momentum calculation
        
        L(:,j) = cross([cm.x(j);cm.y(j);cm.z(j)],P(:,j)) + robot.parts(it).R_mat*I_loc(:,:,j)*robot.parts(it).R_mat'*w(:,it);    %Angular momentum
        
        j = j+1;
        it = it+1;
        
    end
    
    
    com_final.x = sum(cm.x.*cm.mass)/sum(cm.mass);
    com_final.y = sum(cm.y.*cm.mass)/sum(cm.mass);
    com_final.z = sum(cm.z.*cm.mass)/sum(cm.mass);
    P_final = sum(P,2);
    L_final = sum(L,2);
    
    cm.x(26)=0;cm.y(26)=0;cm.z(26)=0;
    I_loc(:,:,26) = eye(3);
    cm.mass(26) = 0;
    P(:,26) = [0;0;0];
    L(:,26) = [0;0;0];
    pt(:,26) = [0;0;0;1];
    
    
    j=1;
    
    for i = 1:28
        robot.parts(i).mass = cm.mass(j);
        robot.parts(i).I = I_loc(:,:,j);
        robot.parts(i).com_g =[cm.x(j),cm.y(j),cm.z(j)];
        robot.parts(i).com_l =[pt(1,j),pt(2,j),pt(3,j)];
        robot.parts(i).P = P(:,j);
        robot.parts(i).L = L(:,j);
        if(i==16 || i==21 || i==28)
            robot.parts(i).mass = 0;
            robot.parts(i).I = zeros(3,3);
            robot.parts(i).P = [0;0;0];
            robot.parts(i).L = [0;0;0];
            robot.parts(i).com_g =[0,0,0];
            robot.parts(i).com_l =[0,0,0];
        else
            j=j+1;
        end
    end
    
    
end

if(base == 'hips')
    
    theta1 = theta1+theta(1:9);
    theta2 = theta2+theta(10:14);
    theta3 = theta3+theta(15:21);
    theta4 = theta4+theta(22:28);
    theta_t  = [theta1,theta2,theta3,theta4];
    
    theta(10)=theta(5);
    theta(15)=theta(1);
    theta(22)=theta(1);
    
    
    if(move_base == 1)
        Tr_mat = T_mat;
    else
        Tr_mat = eye(4);
    end
    
    
    % For 1 - 10
    T(:,:,1) = Tr_mat*Hom_Trans(alpha1(1),0,0,theta1(1)); %T_1_2
    T(:,:,2) = Hom_Trans(alpha1(2),-d8,0,theta1(2)); %T_2_3
    T(:,:,3) = Hom_Trans(alpha1(3),0,l8+l9,theta1(3)); %T_3_4
    T(:,:,4) = Hom_Trans(alpha1(4),0,-d9,theta1(4)); %T_4_5
    T(:,:,5) = Hom_Trans(alpha1(5),l11,-l13,theta1(5)); %T_5_6
    T(:,:,6) = Hom_Trans(alpha1(6),0,-d14,theta1(6)); %T_6_7
    T(:,:,7) = Hom_Trans(alpha1(7),0,0,theta1(7)); %T_7_8
    T(:,:,8) = Hom_Trans(alpha1(8),d5,-(l15+l16),theta1(8)); %T_8_9
    T(:,:,9) = Hom_Trans(alpha1(9),-l17,0,theta1(9)); %T_9_10
    
    %For 11 - 15
    T1(:,:,1) = Hom_Trans(alpha2(1),l11,l13,theta2(1)); %T_10_13'
    T1(:,:,2) = Hom_Trans(alpha2(2),0,d14,theta2(2)); %T_13'_14'
    T1(:,:,3) = Hom_Trans(alpha2(3),0,0,theta2(3)); %T_14'_15'
    T1(:,:,4) = Hom_Trans(alpha2(4),d5,-(l15+l16),theta2(4)); %T_15'_16'
    T1(:,:,5) = Hom_Trans(alpha2(5),-l17,0,theta2(5)); %T_16'_17'
    
    %For 16 - 22
    T2(:,:,1) = Tr_mat*Hom_Trans(alpha3(1),sqrt(l6^2 + d5^2),-d6,theta3(1)); %T_5_6
    T2(:,:,2) = Hom_Trans(alpha3(2),-l5,0,theta3(2)); %T_4_5
    T2(:,:,3) = Hom_Trans(alpha3(3),d4,-l4,theta3(3)); %T_3_4
    T2(:,:,4) = Hom_Trans(alpha3(4),l3,d3,theta3(4)); %T_2_3
    T2(:,:,5) = Hom_Trans(alpha3(5),l2,0,theta3(5)); %T_1_2
    T2(:,:,6) = Hom_Trans(alpha3(6),l1,0,theta3(6)); %T_0_1
    T2(:,:,7) = Hom_Trans(alpha3(7),l0,0,theta3(7)); %T_w_0
    
    %For 23 - 29
    T3(:,:,1) = Tr_mat*Hom_Trans(alpha4(1),sqrt(l6^2 + d5^2),-d6,theta4(1)); %T_5_6
    T3(:,:,2) = Hom_Trans(alpha4(2),l5,0,theta4(2)); %T_4_5
    T3(:,:,3) = Hom_Trans(alpha4(3),d4,-l4,theta4(3)); %T_3_4
    T3(:,:,4) = Hom_Trans(alpha4(4),l3,-d3,theta4(4)); %T_2_3
    T3(:,:,5) = Hom_Trans(alpha4(5),l2,0,theta4(5)); %T_1_2
    T3(:,:,6) = Hom_Trans(alpha4(6),l1,0,theta4(6)); %T_0_1
    T3(:,:,7) = Hom_Trans(alpha4(7),l0,0,theta4(7)); %T_w_0
    
    for i=1:9
        if(i==1)
            T_m(:,:,i) = T(:,:,i);
            temp = Hom_Trans(alpha1(1),0,0,theta1(1));
            b_t(:,i+1) = temp*[0;0;0;1];
            br_t(:,i+1) = inv(temp)*[0;0;0;1];
            a_t(:,i+1) = temp(1:3,1:3)*[0;0;1];
            ar_t(:,i+1) = temp(1:3,1:3)'*[0;0;1];
        else
            T_m(:,:,i) = T_m(:,:,i-1)*T(:,:,i);
            b_t(:,i+1) = T(:,:,i)*[0;0;0;1];
            br_t(:,i+1) = inv(T(:,:,i))*[0;0;0;1];
            a_t(:,i+1) = T(1:3,1:3,i)*[0;0;1];
            ar_t(:,i+1) = T(1:3,1:3,i)'*[0;0;1];
        end
        R(:,:,i) = T_m(1:3,1:3,i);
        R_t(:,:,i+1) = R(:,:,i);
    end
    
    for i=1:6
        if(i==1)
            T1_m(:,:,i) = T_m(:,:,4);
            R1(:,:,i) = T1_m(1:3,1:3,i);
        else
            T1_m(:,:,i) = T1_m(:,:,i-1)*T1(:,:,i-1);
            R1(:,:,i) = T1_m(1:3,1:3,i);
            R_t(:,:,i+10) = R1(:,:,i);
            b_t(:,i+10) = T1(:,:,i-1)*[0;0;0;1];
            br_t(:,i+10) = inv(T1(:,:,i-1))*[0;0;0;1];
            a_t(:,i+10) = T1(1:3,1:3,i-1)*[0;0;1];
            ar_t(:,i+10) = T1(1:3,1:3,i-1)'*[0;0;1];
        end
    end
    
    for i=1:7
        if(i==1)
            T2_m(:,:,i) = T2(:,:,i);
            temp = Hom_Trans(alpha3(1),sqrt(l6^2 + d5^2),-d6,theta3(1));
            b_t(:,i+15) = temp*[0;0;0;1];
            br_t(:,i+15) = inv(temp)*[0;0;0;1];
            a_t(:,i+15) = temp(1:3,1:3)*[0;0;1];
            ar_t(:,i+15) = temp(1:3,1:3)'*[0;0;1];
        else
            T2_m(:,:,i) = T2_m(:,:,i-1)*T2(:,:,i);
            b_t(:,i+15) = T2(:,:,i)*[0;0;0;1];
            br_t(:,i+15) = inv(T2(:,:,i))*[0;0;0;1];
            a_t(:,i+15) = T2(1:3,1:3,i)*[0;0;1];
            ar_t(:,i+15) = T2(1:3,1:3,i)'*[0;0;1];
        end
        R2(:,:,i) = T2_m(1:3,1:3,i);
        R_t(:,:,i+15) = R2(:,:,i);
    end
    
    for i=1:7
        if(i==1)
            T3_m(:,:,i) = T3(:,:,i);
            temp = Hom_Trans(alpha4(1),sqrt(l6^2 + d5^2),-d6,theta4(1));
            b_t(:,i+22) = temp*[0;0;0;1];
            br_t(:,i+22) = inv(temp)*[0;0;0;1];
            a_t(:,i+22) = temp(1:3,1:3)*[0;0;1];
            ar_t(:,i+22) = temp(1:3,1:3)'*[0;0;1];
        else
            T3_m(:,:,i) = T3_m(:,:,i-1)*T3(:,:,i);
            b_t(:,i+22) = T3(:,:,i)*[0;0;0;1];
            br_t(:,i+22) = inv(T3(:,:,i))*[0;0;0;1];
            a_t(:,i+22) = T3(1:3,1:3,i)*[0;0;1];
            ar_t(:,i+22) = T3(1:3,1:3,i)'*[0;0;1];
        end
        R3(:,:,i) = T3_m(1:3,1:3,i);
        R_t(:,:,i+22) = R3(:,:,i);
    end
    %Calculating Jacobian
    pts(:,1) = Tr_mat*[0;0;0;1];
    b_t(:,1) = [0;0;0;1];
    br_t(:,1) = [0;0;0;1];
    
    R_t(:,:,1) = Tr_mat(1:3,1:3,:);
    
    u(:,1) = R_t(:,:,1)*[0;0;1];
    a_t(:,1) = [0;0;1];
    ar_t(:,1) = [0;0;1];
    
    for i=1:9
        pts(:,i+1) = T_m(:,:,i)*[0;0;0;1];
        u(:,i+1) = R(:,:,i)*[0;0;1];
    end
    
    for i=1:6
        pts1(:,i) = T1_m(:,:,i)*[0;0;0;1];
        u1(:,i) = R1(:,:,i)*[0;0;1];
    end
    
    for i=1:7
        pts2(:,i) = T2_m(:,:,i)*[0;0;0;1];
        u2(:,i) = R2(:,:,i)*[0;0;1];
    end
    
    for i=1:7
        pts3(:,i) = T3_m(:,:,i)*[0;0;0;1];
        u3(:,i) = R3(:,:,i)*[0;0;1];
    end
    
    p = [pts, pts1(:,2:6), pts2, pts3];
    u = [u, u1(:,2:6), u2, u3];
    
    draw_pts = p;
    draw_pts(:,3) = T_m(:,:,2)*[0;0;l8;1];
    draw_pts(:,4) = T_m(:,:,3)*[0;0;-d9;1];
    
    for i=1:29
        robot.parts(i).axis_loc = p(1:3,i);
        robot.parts(i).joint_loc = draw_pts(1:3,i);
        robot.parts(i).Z_w = u(1:3,i);
        robot.parts(i).R_mat = R_t(:,:,i);
        robot.parts(i).b = b_t(1:3,i);
        robot.parts(i).br = br_t(1:3,i);
        robot.parts(i).a = a_t(1:3,i);
        robot.parts(i).ar = ar_t(1:3,i);
    end
    
    
    T = [0 0 -1 0;0 1 0 0; 1 0 0 483.898;0 0 0 1];
    T=eye(4);
    for i=1:29
        draw_pts(:,i) = T*draw_pts(:,i);
    end
    robot.draw = draw_pts(1:3,:);
    
    
    idx = [9 14 17 15 16 22 24 23 25 18 20 19 21 11 12 5 6 8 7 10 13 3 4 2 1];
    
    cm.mass = com.mass(idx);
    cm.I(:,:,:) = com.I(:,:,idx);
    
    x = com.x(idx);
    y = com.y(idx);
    z = com.z(idx);
    
    It = cm.mass(1)*([x(1);y(1);z(1)]'*[x(1);y(1);z(1)] - [x(1);y(1);z(1)]*[x(1);y(1);z(1)]'); % Translation to local frame
    I_loc(:,:,1) = It + cm.I(:,:,1);
    
    
    for j=1:25
        
        if(j==1)
            
            tmp = Hom_Trans(0,0,0,-90);
            pt(:,j) = tmp*[x(j);y(j);z(j);1];%1
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = tmp(1:3,1:3)*I_loc(:,:,j)*tmp(1:3,1:3)';
            
        elseif(j==15)
            
            tmp = Hom_Trans(90,0,0,-90);
            pt(:,j) = tmp*[x(j);y(j);z(j);1];%5
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = tmp(1:3,1:3)*I_loc(:,:,j)*tmp(1:3,1:3)';
            
        elseif(j==4)
            
            pt(:,j) = [x(j);y(j);z(j);1]; %6,10
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);                                                            %Rotation not needed here
            
        elseif(j==14)
            
            p = [x(j);y(j);z(j)];
            R = [cosd(-114.0254) -sind(-114.0254) 0;
                sind(-114.0254)  cosd(-114.0254)  0;
                0               0             1;]; %7
            pt(:,j) = [R*p;1];
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = R*I_loc(:,:,j)*R';
            
        elseif(j==3)
            
            tmp = Hom_Trans(90,0,0,90);
            pt(:,j) = tmp*[x(j);y(j);z(j);1];%9
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = tmp(1:3,1:3)*I_loc(:,:,j)*tmp(1:3,1:3)';
            
        elseif(j==7 || j==11 || j==25 || j==19)
            
            tmp = Hom_Trans(0,0,0,90);
            pt(:,j) = tmp*[x(j);y(j);z(j);1];
            %pt(:,j) = [-y(j);x(j);z(j);1];%13,17,25
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = tmp(1:3,1:3)*I_loc(:,:,j)*tmp(1:3,1:3)';
            
        elseif(j==8 || j==12 || j==21)
            
            tmp = Hom_Trans(90,0,0,0);
            pt(:,j) = tmp*[x(j);y(j);z(j);1];
            %pt(:,j) = [x(j);-z(j);y(j);1];%14,18,21
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = tmp(1:3,1:3)*I_loc(:,:,j)*tmp(1:3,1:3)';
            
        elseif(j==9 || j==13 || j==22 || j==16)
            
            tmp = Hom_Trans(0,0,0,0);
            tmp(1:3,1:3) = [0 0 1;0 1 0;-1 0 0]*[-1 0 0;0 -1 0; 0 0 1];
            pt(:,j) = tmp*[x(j);y(j);z(j);1];
            %pt(:,j) = [z(j);-y(j);x(j);1];%15,19,22
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = tmp(1:3,1:3)*I_loc(:,:,j)*tmp(1:3,1:3)';
            
        elseif(j==20)
            p = [x(j);y(j);z(j)];
            R = [cosd(-65.9746) -sind(-65.9746) 0;
                sind(-65.9746)  cosd(-65.9746)  0;
                0               0             1;];%20
            pt(:,j) = [R*p;1];
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = R*I_loc(:,:,j)*R';
            
        elseif(j==23 || j==24 || j==18 || j==17)
            
            tmp = Hom_Trans(0,0,0,0);
            tmp(1:3,1:3) = [1 0 0;0 0 -1;0 1 0]*[0 -1 0;1 0 0; 0 0 1];
            pt(:,j) = tmp*[x(j);y(j);z(j);1];
            %pt(:,j) = [-y(j);-z(j);x(j);1];%23,24
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = tmp(1:3,1:3)*I_loc(:,:,j)*tmp(1:3,1:3)';
            
        else
            
            tmp = Hom_Trans(0,0,0,0);
            tmp(1:3,1:3) = [1 0 0;0 0 1;0 -1 0]*[0 1 0;-1 0 0; 0 0 1];
            pt(:,j) = tmp*[x(j);y(j);z(j);1];
            %pt(:,j) = [y(j);z(j);x(j);1]; % 2,5,6,10,18,17,16
            
            It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
            I_loc(:,:,j) = It + cm.I(:,:,j);
            
            I_loc(:,:,j) = tmp(1:3,1:3)*I_loc(:,:,j)*tmp(1:3,1:3)';
            
        end
        
        
    end
    
    pt(1:3,1) = rotVecAroundArbAxis([pt(1,1),pt(2,1),pt(3,1)],robot.parts(1).Z_w',theta(1));
    tmp = Tr_mat*[pt(1,1);pt(2,1);pt(3,1);1];
    
    cm.x(1) = tmp(1);
    cm.y(1) = tmp(2);
    cm.z(1) = tmp(3);
    
    %     for i = 1:29
    %         robot.parts(i).v =[0;0;0];
    %         robot.parts(i).w =[0;0;0];
    %     end
    
    [v,w] = standing_vel(dq);
    
    c1 = v(:,1)+ cross(w(:,1),robot.parts(1).R_mat*[pt(1,1);pt(2,1);pt(3,1)]);
    P(:,1) = cm.mass(1)*c1;                                                                 %Linear momentum calculation
    
    L(:,1) = cross([cm.x(1);cm.y(1);cm.z(1)],P(:,1)) + robot.parts(1).R_mat*I_loc(:,:,1)*robot.parts(1).R_mat'*w(:,1);    %Angular momentum
    
    j=2;
    it=2;
    
    for i=1:8
        
        pt(1:3,j) = rotVecAroundArbAxis([pt(1,j),pt(2,j),pt(3,j)],robot.parts(it).Z_w',theta(it));
        tmp = T_m(:,:,i)*pt(:,j);
        
        cm.x(j) = tmp(1);
        cm.y(j) = tmp(2);
        cm.z(j) = tmp(3);
        
        c1 = v(:,it)+ cross(w(:,it),robot.parts(it).R_mat*[pt(1,j);pt(2,j);pt(3,j)]);
        P(:,j) = cm.mass(j)*c1;                                                                 %Linear momentum calculation
        
        L(:,j) = cross([cm.x(j);cm.y(j);cm.z(j)],P(:,j)) + robot.parts(it).R_mat*I_loc(:,:,j)*robot.parts(it).R_mat'*w(:,it);    %Angular momentum
        
        j = j+1;
        it = it+1;
    end
    
    it = it +1;
    
    for i=2:5
        
        pt(1:3,j) = rotVecAroundArbAxis([pt(1,j),pt(2,j),pt(3,j)],robot.parts(it).Z_w',theta(it));
        tmp = T1_m(:,:,i)*pt(:,j);
        
        cm.x(j) = tmp(1);
        cm.y(j) = tmp(2);
        cm.z(j) = tmp(3);
        
        c1 = v(:,it)+ cross(w(:,it),robot.parts(it).R_mat*[pt(1,j);pt(2,j);pt(3,j)]);
        P(:,j) = cm.mass(j)*c1;                                                                 %Linear momentum calculation
        
        L(:,j) = cross([cm.x(j);cm.y(j);cm.z(j)],P(:,j)) + robot.parts(it).R_mat*I_loc(:,:,j)*robot.parts(it).R_mat'*w(:,it);    %Angular momentum
        
        j = j+1;
        it = it+1;
    end
    
    it = it+1;
    
    for i=1:6
        
        pt(1:3,j) = rotVecAroundArbAxis([pt(1,j),pt(2,j),pt(3,j)],robot.parts(it).Z_w',theta(it));
        tmp = T2_m(:,:,i)*pt(:,j);
        
        cm.x(j) = tmp(1);
        cm.y(j) = tmp(2);
        cm.z(j) = tmp(3);
        
        c1 = v(:,it)+ cross(w(:,it),robot.parts(it).R_mat*[pt(1,j);pt(2,j);pt(3,j)]);
        P(:,j) = cm.mass(j)*c1;                                                                 %Linear momentum calculation
        
        L(:,j) = cross([cm.x(j);cm.y(j);cm.z(j)],P(:,j)) + robot.parts(it).R_mat*I_loc(:,:,j)*robot.parts(it).R_mat'*w(:,it);    %Angular momentum
        
        j = j+1;
        it = it+1;
    end
    
    it = it+1;
    
    for i=1:6
        
        pt(1:3,j) = rotVecAroundArbAxis([pt(1,j),pt(2,j),pt(3,j)],robot.parts(it).Z_w',theta(it));
        tmp = T3_m(:,:,i)*pt(:,j);
        
        cm.x(j) = tmp(1);
        cm.y(j) = tmp(2);
        cm.z(j) = tmp(3);
        
        c1 = v(:,it)+ cross(w(:,it),robot.parts(it).R_mat*[pt(1,j);pt(2,j);pt(3,j)]);
        P(:,j) = cm.mass(j)*c1;                                                                 %Linear momentum calculation
        
        L(:,j) = cross([cm.x(j);cm.y(j);cm.z(j)],P(:,j)) + robot.parts(it).R_mat*I_loc(:,:,j)*robot.parts(it).R_mat'*w(:,it);    %Angular momentum
        
        j = j+1;
        it = it+1;
    end
    com_final.x = sum(cm.x.*cm.mass)/sum(cm.mass);
    com_final.y = sum(cm.y.*cm.mass)/sum(cm.mass);
    com_final.z = sum(cm.z.*cm.mass)/sum(cm.mass);
    P_final = sum(P,2);
    L_final = sum(L,2);
    
    
    cm.x(26)=0;cm.y(26)=0;cm.z(26)=0;
    I_loc(:,:,26) = eye(3);
    cm.mass(26) = 0;
    P(:,26) = [0;0;0];
    L(:,26) = [0;0;0];
    pt(:,26) = [0;0;0;1];
    
    %RIR'           %orientation to the global frame
    
    j=1;
    
    for i = 1:29
        robot.parts(i).mass = cm.mass(j);
        robot.parts(i).I = I_loc(:,:,j);
        robot.parts(i).com_g =[cm.x(j),cm.y(j),cm.z(j)];
        robot.parts(i).com_l =[pt(1,j),pt(2,j),pt(3,j)];
        robot.parts(i).P = P(:,j);
        robot.parts(i).L = L(:,j);
        if(i==10 || i==15 || i==22 || i==29)
            robot.parts(i).mass = 0;
            robot.parts(i).I = zeros(3,3);
            robot.parts(i).P = [0;0;0];
            robot.parts(i).L = [0;0;0];
        else
            j=j+1;
        end
    end
    
    
end
end

