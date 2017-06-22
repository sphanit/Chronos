function [com_final,cm] = ForwKin_prev(theta,T_mat,dq)
global robot
global base
global move_base
load model_data
load com_data_new
cm_data(1).I;

for i=1:25
    com.x(i) = sum(cm_data(i).x.*cm_data(i).mass')/sum(cm_data(i).mass);
    com.y(i) = sum(cm_data(i).y.*cm_data(i).mass')/sum(cm_data(i).mass);
    com.z(i) = sum(cm_data(i).z.*cm_data(i).mass')/sum(cm_data(i).mass);
    com.mass(i) = sum(cm_data(i).mass);
    com.I(:,:,i) = cm_data(i).I;
end

tmp = Hom_Trans(0,0,0,-90);
R_ref = tmp(1:3,1:3);

if(base == 'leg')
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
    T_tran = eye(4);
    T(:,:,2) = Tr_mat*Hom_Trans(alp1(2),l1,0,th1(2)); %T_0_1
    T(:,:,3) = Hom_Trans(alp1(3),l2,0,th1(3)); %T_1_2
    T(:,:,4) = Hom_Trans(alp1(4),l3,-d3,th1(4)); %T_2_3
    T(:,:,5) = Hom_Trans(alp1(5),d4,0,th1(5)); %T_3_4
    T(:,:,6) = Hom_Trans(alp1(6),l5,l4,th1(6)); %T_4_5
    T(:,:,7) = Hom_Trans(alp1(7),sqrt(l6^2 + d5^2),d6,th1(7)); %T_5_6
    
    T(:,:,8) = Hom_Trans(alp1(8),0,0,th1(8)); %T_6_7
    
    
    T(:,:,9) = Hom_Trans(alp1(9),-d8,0,th1(9)); %T_7_8
    T(:,:,10) = Hom_Trans(alp1(10),0,l8+l9,th1(10)); %T_8_9
    T(:,:,11) = Hom_Trans(alp1(11),0,-d9,th1(11)); %T_9_10
    
    %For 13 - 17
    T(:,:,12) = Hom_Trans(alp1(12),l11,-l13,th1(12)); %T_10_13
    T(:,:,13) = Hom_Trans(alp1(13),0,-d14,th1(13)); %T_13_14
    T(:,:,14) = Hom_Trans(alp1(14),0,0,th1(14)); %T_14_15
    T(:,:,15) = Hom_Trans(alp1(15),d5,-(l15+l16),th1(15)); %T_15_16
    T(:,:,16) = Hom_Trans(alp1(16),-l17,0,th1(16)); %T_16_17
    
    %For 13' - 17'
    T1(:,:,1) = Hom_Trans(alp2(1),l11,l13,th2(1)); %T_10_13'
    T1(:,:,2) = Hom_Trans(alp2(2),0,d14,th2(2)); %T_13'_14'
    T1(:,:,3) = Hom_Trans(alp2(3),0,0,th2(3)); %T_14'_15'
    T1(:,:,4) = Hom_Trans(alp2(4),d5,-(l15+l16),th2(4)); %T_15'_16'
    T1(:,:,5) = Hom_Trans(alp2(5),-l17,0,th2(5)); %T_16'_17'
    
    %For 5' - 00'
    T2(:,:,1) = Hom_Trans(alp3(1),sqrt(l6^2 + d5^2),-d6,th3(1)); %T_5_6
    T2(:,:,2) = Hom_Trans(alp3(2),l5,0,th3(2)); %T_4_5
    T2(:,:,3) = Hom_Trans(alp3(3),d4,-l4,th3(3)); %T_3_4
    T2(:,:,4) = Hom_Trans(alp3(4),l3,-d3,th3(4)); %T_2_3
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
    draw_pts(:,1) = (T(:,:,1)^-1)*[0;0;0;1];
    draw_pts(:,2) = [0;0;0;1];
    
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
    T=eye(4);
    for i=1:29
        draw_pts(:,i) = T*draw_pts(:,i);
    end
    robot.draw = draw_pts(1:3,:);
    
    idx = [7 8 6 5 12 10 9 14 17 15 16 22 24 23 25 18 20 19 21 11 13 3 4 2 1]; 

    cm.mass = com.mass(idx);
    cm.I(:,:,:) = com.I(:,:,idx);
    
    x = com.x(idx);
    y = com.y(idx);
    z = com.z(idx);
    
    T_ref = eye(4);
    T_ref(1:3,1:3) = R_ref;
    tmp = Tr_mat*T_ref*T_tran*[x(1);y(1);z(1);1];
    
    cm.x(1) = tmp(1);
    cm.y(1) = tmp(2);
    cm.z(1) = tmp(3);
    
    It = cm.mass(1)*([x(1);y(1);z(1)]'*[x(1);y(1);z(1)] - [x(1);y(1);z(1)]*[x(1);y(1);z(1)]'); % Translation to local frame
    I_loc(:,:,1) = It + cm.I(:,:,1);
    
    for i = 1:29
        robot.parts(i).v =[0;0;0];
        robot.parts(i).w =[0;0;0];
    end
    
    [v,w] = walking_vel(dq);
    
    c1 = v(:,1)+ cross(w(:,1),R_ref*[x(1);y(1);z(1)]);
    P(:,1) = cm.mass(1)*c1;                                                                 %Linear momentum calculation
    
    L(:,1) = cross([cm.x(1);cm.y(1);cm.z(1)],P(:,1)) + R_ref*I_loc(:,:,1)*R_ref'*w(:,1);    %Angular momentum
    
    j = 2;
    it = 2;
    for i=1:14
        T_ref = T_tran*T_m(:,:,i);
        T_ref(1:3,1:3) = R_ref;
        cm_pt = T_ref*[x(j);y(j);z(j);1];
        cm.x(j) = cm_pt(1);
        cm.y(j) = cm_pt(2);
        cm.z(j) = cm_pt(3);
        
        It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
        I_loc(:,:,j) = It + cm.I(:,:,j);
        
        c1 = v(:,it)+ cross(w(:,it),R_ref*[x(j);y(j);z(j)]);
        P(:,j) = cm.mass(j)*c1;                                                                 %Linear momentum calculation
        
        L(:,j) = cross([cm.x(j);cm.y(j);cm.z(j)],P(:,j)) + R_ref*I_loc(:,:,j)*R_ref'*w(:,it);    %Angular momentum
        
        if (i==4)
            cm.x(j) = cm_pt(1)+l4;
        else
            cm.x(j) = cm_pt(1);
        end
        
        j = j+1;
        it = it+1;
        
    end
    
    for i=2:5
        T_ref = T_tran*T1_m(:,:,i);
        T_ref(1:3,1:3) = R_ref;
        cm_pt = T_ref*[x(j);y(j);z(j);1];
        cm.x(j) = cm_pt(1);
        cm.y(j) = cm_pt(2);
        cm.z(j) = cm_pt(3);
        
        It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
        I_loc(:,:,j) = It + cm.I(:,:,j);
        
        c1 = v(:,it)+ cross(w(:,it),R_ref*[x(j);y(j);z(j)]);
        P(:,j) = cm.mass(j)*c1;                                                                 %Linear momentum calculation
        
        L(:,j) = cross([cm.x(j);cm.y(j);cm.z(j)],P(:,j)) + R_ref*I_loc(:,:,j)*R_ref'*w(:,it);    %Angular momentum
        
        j = j+1;
        it = it+1;
    end
    
    for i=2:7
        T_ref = T_tran*T2_m(:,:,i);
        T_ref(1:3,1:3) = R_ref;
        cm_pt = T_ref*[x(j);y(j);z(j);1];
        cm.x(j) = cm_pt(1);
        cm.y(j) = cm_pt(2);
        cm.z(j) = cm_pt(3);
        
        It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
        I_loc(:,:,j) = It + cm.I(:,:,j);
        
        c1 = v(:,it)+ cross(w(:,it),R_ref*[x(j);y(j);z(j)]);
        P(:,j) = cm.mass(j)*c1;                                                                 %Linear momentum calculation
        
        L(:,j) = cross([cm.x(j);cm.y(j);cm.z(j)],P(:,j)) + R_ref*I_loc(:,:,j)*R_ref'*w(:,it);    %Angular momentum
        
        j = j+1;
        it = it+1;
    end
    
    
    com_final.x = sum(cm.x.*cm.mass)/sum(cm.mass);
    com_final.y = sum(cm.y.*cm.mass)/sum(cm.mass);
    com_final.z = sum(cm.z.*cm.mass)/sum(cm.mass);
    
    
end

if(base == 'hip')
    
    theta1 = theta1+theta(1:9);
    theta2 = theta2+theta(10:14);
    theta3 = theta3+theta(15:21);
    theta4 = theta4+theta(22:28);
    theta_t  = [theta1,theta2,theta3,theta4];
    
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
    
    for i=1:29
        draw_pts(:,i) = T*draw_pts(:,i);
    end
    robot.draw = draw_pts(1:3,:);
    
    %COM caculation
    
    idx = [9 14 17 15 16 22 24 23 25 18 20 19 21 11 12 5 6 8 7 10 13 3 4 2 1];
    cm.mass = com.mass(idx);
    cm.I(:,:,:) = com.I(:,:,idx);
    
    x = com.x(idx);
    y = com.y(idx);
    z = com.z(idx);
    
    T_ref = eye(4);
    T_ref(1:3,1:3) = R_ref;
    tmp = Tr_mat*T_ref*[x(1);y(1);z(1);1];
    
    tmp = rotVecAroundArbAxis([tmp(1),tmp(2),tmp(3)],robot.parts(1).Z_w',theta(1)); 
        
    cm.x(1) = tmp(1);
    cm.y(1) = tmp(2);
    cm.z(1) = tmp(3);
   
    
    It = cm.mass(1)*([x(1);y(1);z(1)]'*[x(1);y(1);z(1)] - [x(1);y(1);z(1)]*[x(1);y(1);z(1)]'); % Translation to local frame
    I_loc(:,:,1) = It + cm.I(:,:,1);
    
    for i = 1:29
        robot.parts(i).v =[0;0;0];
        robot.parts(i).w =[0;0;0];
    end
    
    [v,w] = walking_vel(dq);
    
    c1 = v(:,1)+ cross(w(:,1),R_ref*[x(1);y(1);z(1)]);
    P(:,1) = cm.mass(1)*c1;                                                                 %Linear momentum calculation
    
    L(:,1) = cross([cm.x(1);cm.y(1);cm.z(1)],P(:,1)) + R_ref*I_loc(:,:,1)*R_ref'*w(:,1);    %Angular momentum
    
    j=2;
    it = 2;
    
    %Icm + It       %translation to the local frame
    for i=1:8
        T_ref = T_m(:,:,i);
        T_ref(1:3,1:3) = R_ref;
        tmp = T_ref*[x(j);y(j);z(j);1];
        tmp = rotVecAroundArbAxis([tmp(1),tmp(2),tmp(3)],robot.parts(it).Z_w',theta(it));
        
        cm.x(j) = tmp(1);
        cm.y(j) = tmp(2);
        cm.z(j) = tmp(3);
%                 cm.x(j) = cm_pt(1);
%                 cm.y(j) = cm_pt(2);
%                 cm.z(j) = cm_pt(3);
         
        It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
        I_loc(:,:,j) = It + cm.I(:,:,j);
        
        c1 = v(:,it)+ cross(w(:,it),R_ref*[x(j);y(j);z(j)]);
        P(:,j) = cm.mass(j)*c1;                                                                 %Linear momentum calculation
        
        L(:,j) = cross([cm.x(j);cm.y(j);cm.z(j)],P(:,j)) + R_ref*I_loc(:,:,j)*R_ref'*w(:,it);    %Angular momentum
        
        j = j+1;
        it = it+1;
    end
        it=it+1;
        
    for i=2:5
        T_ref = T1_m(:,:,i);
        T_ref(1:3,1:3) = R_ref;
        
        tmp = T_ref*[x(j);y(j);z(j);1];
        tmp = rotVecAroundArbAxis([tmp(1),tmp(2),tmp(3)],robot.parts(it).Z_w',theta(it));
        
        cm.x(j) = tmp(1);
        cm.y(j) = tmp(2);
        cm.z(j) = tmp(3);
%         cm_pt = T_ref*[x(j);y(j);z(j);1];
%         cm.x(j) = cm_pt(1);
%         cm.y(j) = cm_pt(2);
%         cm.z(j) = cm_pt(3);
       
        It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
        I_loc(:,:,j) = It + cm.I(:,:,j);
        c1 = v(:,it)+ cross(w(:,it),R_ref*[x(j);y(j);z(j)]);
        P(:,j) = cm.mass(j)*c1;                                                                 %Linear momentum calculation
        
        L(:,j) = cross([cm.x(j);cm.y(j);cm.z(j)],P(:,j)) + R_ref*I_loc(:,:,j)*R_ref'*w(:,it);    %Angular momentum
        
        j = j+1;
        it = it+1;
    end
        it=it+1;
    
    for i=1:6
        T_ref = T2_m(:,:,i);
        T_ref(1:3,1:3) = R_ref;
        
        tmp = T_ref*[x(j);y(j);z(j);1];
        tmp = rotVecAroundArbAxis([tmp(1),tmp(2),tmp(3)],robot.parts(it).Z_w',theta(it));
        
        cm.x(j) = tmp(1);
        cm.y(j) = tmp(2);
        cm.z(j) = tmp(3);
%         cm_pt = T_ref*[x(j);y(j);z(j);1];
%         cm.x(j) = cm_pt(1);
%         cm.y(j) = cm_pt(2);
%         cm.z(j) = cm_pt(3);
                
        It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
        I_loc(:,:,j) = It + cm.I(:,:,j);
        
        c1 = v(:,it)+ cross(w(:,it),R_ref*[x(j);y(j);z(j)]);
        P(:,j) = cm.mass(j)*c1;                                                                 %Linear momentum calculation
        
        L(:,j) = cross([cm.x(j);cm.y(j);cm.z(j)],P(:,j)) + R_ref*I_loc(:,:,j)*R_ref'*w(:,it);    %Angular momentum
        
        j = j+1;
        it = it+1;
    end
        it=it+1;
    
    for i=1:6
        T_ref = T3_m(:,:,i);
        T_ref(1:3,1:3) = R_ref;
        
        tmp = T_ref*[x(j);y(j);z(j);1];
        tmp = rotVecAroundArbAxis([tmp(1),tmp(2),tmp(3)],robot.parts(it).Z_w',theta(it));
        
        cm.x(j) = tmp(1);
        cm.y(j) = tmp(2);
        cm.z(j) = tmp(3);
%         cm_pt = T_ref*[x(j);y(j);z(j);1];
%         cm.x(j) = cm_pt(1);
%         cm.y(j) = cm_pt(2);
%         cm.z(j) = cm_pt(3);
                
        It = cm.mass(j)*([x(j);y(j);z(j)]'*[x(j);y(j);z(j)] - [x(j);y(j);z(j)]*[x(j);y(j);z(j)]'); % Translation to local frame
        I_loc(:,:,j) = It + cm.I(:,:,j);
        
        c1 = v(:,it)+ cross(w(:,it),R_ref*[x(j);y(j);z(j)]);
        P(:,j) = cm.mass(j)*c1;                                                                 %Linear momentum calculation
        
        L(:,j) = cross([cm.x(j);cm.y(j);cm.z(j)],P(:,j)) + R_ref*I_loc(:,:,j)*R_ref'*w(:,it);    %Angular momentum
        
        j = j+1;
        it = it+1;
    end
  
    com_final.x = sum(cm.x.*cm.mass)/sum(cm.mass);
    com_final.y = sum(cm.y.*cm.mass)/sum(cm.mass);
    com_final.z = sum(cm.z.*cm.mass)/sum(cm.mass);
    P_final = sum(P,2);
    L_final = sum(L,2);
    sum(cm.x.*cm.mass)
    

    cm.x(26)=0;cm.y(26)=0;cm.z(26)=0;
    I_loc(:,:,26) = eye(3);
    cm.mass(26) = 0;
    x(26)=0;y(26)=0;z(26)=0;
    P(:,26) = [0;0;0];
    L(:,26) = [0;0;0];
    
    %RIR'           %orientation to the global frame
    
    j=1;
    for i = 1:29
        robot.parts(i).mass = cm.mass(j);
        robot.parts(i).I = I_loc(:,:,j);
        robot.parts(i).com_g =[cm.x(j),cm.y(j),cm.z(j)];
        robot.parts(i).com_lg =R_ref*[x(j);y(j);z(j)];
        robot.parts(i).com_l =[0,0,0];
        robot.parts(i).P = P(:,j);
        robot.parts(i).L = L(:,j);
        if(i==10 || i==15 || i==22 || i==29)
            robot.parts(i).mass = 0;
            robot.parts(i).I = eye(3);
            robot.parts(i).P = [0;0;0];
            robot.parts(i).L = [0;0;0];
        else
            j=j+1;
        end
    end
    
end
end

% %     cm.x(1) = y(1);
% %     cm.y(1) = -x(1);
% %     cm.z(1) = z(1);
% %     
% %     pt(:,1) = [cm.x(1);cm.y(1);cm.z(1);1];
% %     
% %     for j=2:25
% %         
% %         if(j==5)
% %             
% %             pt(:,j) = [-z(j);-x(j);y(j);1];%5
% %             
% %             
% %         elseif(j==6 || j==10)
% %             
% %             pt(:,j) = [x(j);y(j);z(j);1]; %6,10
% %             
% %         elseif(j==7)
% %             
% %             p = [x(j);y(j);z(j)];
% %             R = [cosd(-65.9746) -sind(-65.9746) 0;
% %                 sind(-65.9746)  cosd(-65.9746)  0;
% %                 0               0             1;]; %7
% %             pt(:,j) = [R*p;1];
% %             
% %         elseif(j==9)
% %             pt(:,j) = [z(j);x(j);y(j);1];%9
% %             
% %         elseif(j==13 || j==17 || j==25)
% %             
% %             pt(:,j) = [-y(j);x(j);z(j);1];%13,17,25
% %             
% %         elseif(j==14 || j==18 || j==21)
% %             pt(:,j) = [x(j);-z(j);y(j);1];%14,18,21
% %         elseif(j==15 || j==19 || j==22)
% %             pt(:,j) = [z(j);-y(j);x(j);1];%15,19,22
% %         elseif(j==20)
% %             p = [x(j);y(j);z(j)];
% %             R = [cosd(65.9746) -sind(65.9746) 0;
% %                 sind(65.9746)  cosd(65.9746)  0;
% %                 0               0             1;];%20
% %             pt(:,j) = [R*p;1];
% %         elseif(j==23 || j==24)
% %             pt(:,j) = [-y(j);-z(j);x(j);1];%23,24
% %         else
% %             pt(:,j) = [y(j);z(j);x(j);1]; % 2,3,4,8,11,12,16
% %         end
% %         
% %         
% %     end