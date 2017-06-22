function [com,cm,com_final] = cal_com(theta)
load com_data_new

for i=1:25
    com.x(i) = sum(cm_data(i).x.*cm_data(i).mass')/sum(cm_data(i).mass);
    com.y(i) = sum(cm_data(i).y.*cm_data(i).mass')/sum(cm_data(i).mass);
    com.z(i) = sum(cm_data(i).z.*cm_data(i).mass')/sum(cm_data(i).mass);
    com.mass(i) = sum(cm_data(i).mass);
end
global robot
global sim
global base
%base = 'hip';
load model_data.mat
%theta = zeros(1,28);
if(base == 'leg')
    th1(2:16) = th1(2:16) + theta(1:15);
    th2 = th2 + theta(16:20);
    th3 = th3 + theta(21:27);
    
    % For 0 - 17
    T(:,:,1) = Hom_Trans(alp1(1),l0,0,th1(1)); %T_w_0
    T(:,:,2) = Hom_Trans(alp1(2),l1,0,th1(2)); %T_0_1
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
    
    for i=1:14
        if(i==1)
            T_m(:,:,i) = T(:,:,i+1);
        else
            T_m(:,:,i) = T_m(:,:,i-1)*T(:,:,i+1);
        end
        
        R(:,:,i) = T_m(1:3,1:3,i);
    end
    
    
    for i=1:5
        if(i==1)
            T1_m(:,:,i) = T_m(:,:,10);
        else
            T1_m(:,:,i) = T1_m(:,:,i-1)*T1(:,:,i-1);
        end
        R1(:,:,i) = T1_m(1:3,1:3,i);
    end
    
    
    for i=1:7
        if(i==1)
            T2_m(:,:,i) = T_m(:,:,6);
        else
            T2_m(:,:,i) = T2_m(:,:,i-1)*T2(:,:,i-1);
        end
        R2(:,:,i) = T2_m(1:3,1:3,i);
    end
    
    idx = [7 8 6 5 12 10 9 14 17 15 16 22 24 23 25 18 20 19 21 11 13 3 4 2 1];
    cm.mass = com.mass(idx);
    x = com.x(idx);
    y = com.y(idx);
    z = com.z(idx);
    
    cm.x(1) = y(1);
    cm.y(1) = -x(1);
    cm.z(1) = z(1);
    
    pt(:,1) = [cm.x(1);cm.y(1);cm.z(1);1];
    
    for j=2:25
        
        if(j==5)
            
            pt(:,j) = [-z(j);-x(j);y(j);1];%5
            
            
        elseif(j==6 || j==10)
            
            pt(:,j) = [x(j);y(j);z(j);1]; %6,10
            
        elseif(j==7)
            
            p = [x(j);y(j);z(j)];
            R = [cosd(-65.9746) -sind(-65.9746) 0;
                sind(-65.9746)  cosd(-65.9746)  0;
                0               0             1;]; %7
            pt(:,j) = [R*p;1];
            
        elseif(j==9)
            pt(:,j) = [z(j);x(j);y(j);1];%9
            
        elseif(j==13 || j==17 || j==25)
            
            pt(:,j) = [-y(j);x(j);z(j);1];%13,17,25
            
        elseif(j==14 || j==18 || j==21)
            pt(:,j) = [x(j);-z(j);y(j);1];%14,18,21
        elseif(j==15 || j==19 || j==22)
            pt(:,j) = [z(j);-y(j);x(j);1];%15,19,22
        elseif(j==20)
            p = [x(j);y(j);z(j)];
            R = [cosd(65.9746) -sind(65.9746) 0;
                sind(65.9746)  cosd(65.9746)  0;
                0               0             1;];%20
            pt(:,j) = [R*p;1];
        elseif(j==23 || j==24)
            pt(:,j) = [-y(j);-z(j);x(j);1];%23,24
        else
            pt(:,j) = [y(j);z(j);x(j);1]; % 2,3,4,8,11,12,16
        end
        
        
    end
    
    j = 2;
    for i=1:14
        cm_pt = T_m(:,:,i)*pt(:,j);
        if (i==4)
            cm.x(j) = cm_pt(1)+l4;
        else
            cm.x(j) = cm_pt(1);
        end
        cm.y(j) = cm_pt(2);
        cm.z(j) = cm_pt(3);
        j = j+1;
    end
    
    for i=2:5
        cm_pt = T1_m(:,:,i)*pt(:,j);
        cm.x(j) = cm_pt(1);
        cm.y(j) = cm_pt(2);
        cm.z(j) = cm_pt(3);
        j = j+1;
    end
    
    for i=2:7
        cm_pt = T2_m(:,:,i)*pt(:,j);
        cm.x(j) = cm_pt(1);
        cm.y(j) = cm_pt(2);
        cm.z(j) = cm_pt(3);
        j = j+1;
    end
    
    
    com_final.x = sum(cm.x.*cm.mass)/sum(cm.mass);
    com_final.y = sum(cm.y.*cm.mass)/sum(cm.mass);
    com_final.z = sum(cm.z.*cm.mass)/sum(cm.mass);
    
    %     figure
    %     plot3(cm.x,cm.y,cm.z,'ko');
    %     hold on;
    %     plot3(cm.x,cm.y,cm.z,'b');
    %     plot3(com_final.x,com_final.y,com_final.z,'ro');
    %     axis equal;
    %     pause(0.05);
    %     hold off;
    
end

if(base == 'hip')
    
    theta1 = theta1+theta(1:9);
    theta2 = theta2+theta(10:14);
    theta3 = theta3+theta(15:21);
    theta4 = theta4+theta(22:28);
    
    % For 1 - 10
    T(:,:,1) = Hom_Trans(alpha1(1),0,0,theta1(1)); %T_1_2
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
    T2(:,:,1) = Hom_Trans(alpha3(1),sqrt(l6^2 + d5^2),-d6,theta3(1)); %T_5_6
    T2(:,:,2) = Hom_Trans(alpha3(2),-l5,0,theta3(2)); %T_4_5
    T2(:,:,3) = Hom_Trans(alpha3(3),d4,-l4,theta3(3)); %T_3_4
    T2(:,:,4) = Hom_Trans(alpha3(4),l3,d3,theta3(4)); %T_2_3
    T2(:,:,5) = Hom_Trans(alpha3(5),l2,0,theta3(5)); %T_1_2
    T2(:,:,6) = Hom_Trans(alpha3(6),l1,0,theta3(6)); %T_0_1
    T2(:,:,7) = Hom_Trans(alpha3(7),l0,0,theta3(7)); %T_w_0
    
    %For 23 - 29
    T3(:,:,1) = Hom_Trans(alpha4(1),sqrt(l6^2 + d5^2),-d6,theta4(1)); %T_5_6
    T3(:,:,2) = Hom_Trans(alpha4(2),l5,0,theta4(2)); %T_4_5
    T3(:,:,3) = Hom_Trans(alpha4(3),d4,-l4,theta4(3)); %T_3_4
    T3(:,:,4) = Hom_Trans(alpha4(4),l3,-d3,theta4(4)); %T_2_3
    T3(:,:,5) = Hom_Trans(alpha4(5),l2,0,theta4(5)); %T_1_2
    T3(:,:,6) = Hom_Trans(alpha4(6),l1,0,theta4(6)); %T_0_1
    T3(:,:,7) = Hom_Trans(alpha4(7),l0,0,theta4(7)); %T_w_0
    
    for i=1:9
        if(i==1)
            T_m(:,:,i) = T(:,:,i);
        else
            T_m(:,:,i) = T_m(:,:,i-1)*T(:,:,i);
        end
        
        R(:,:,i) = T_m(1:3,1:3,i);
    end
    
    for i=1:6
        if(i==1)
            T1_m(:,:,i) = T_m(:,:,4);
        else
            T1_m(:,:,i) = T1_m(:,:,i-1)*T1(:,:,i-1);
        end
        R1(:,:,i) = T1_m(1:3,1:3,i);
    end
    
    for i=1:7
        if(i==1)
            T2_m(:,:,i) = T2(:,:,i);
        else
            T2_m(:,:,i) = T2_m(:,:,i-1)*T2(:,:,i);
        end
        R2(:,:,i) = T2_m(1:3,1:3,i);
    end
    
    for i=1:7
        if(i==1)
            T3_m(:,:,i) = T3(:,:,i);
        else
            T3_m(:,:,i) = T3_m(:,:,i-1)*T3(:,:,i);
        end
        R3(:,:,i) = T3_m(1:3,1:3,i);
    end
    
    idx = [9 14 17 15 16 22 24 23 25 18 20 19 21 11 12 5 6 8 7 10 13 3 4 2 1];
    cm.mass = com.mass(idx);
    
    x = com.x(idx);
    y = com.y(idx);
    z = com.z(idx);
    
    
    for j=1:25
        
        if(j==1)
            pt(:,j) = [y(j);-x(j);z(j);1];%1
            
        elseif(j==15)
            
            pt(:,j) = [-z(j);-x(j);y(j);1];%5
            
            
        elseif(j==4)
            
            pt(:,j) = [x(j);y(j);z(j);1]; %6,10
            
        elseif(j==14)
            
            p = [x(j);y(j);z(j)];
            R = [cosd(-114.0254) -sind(-114.0254) 0;
                sind(-114.0254)  cosd(-114.0254)  0;
                0               0             1;]; %7
            pt(:,j) = [R*p;1];
            
        elseif(j==3)
            pt(:,j) = [z(j);x(j);y(j);1];%9
            
        elseif(j==7 || j==11 || j==25 || j==19)
            
            pt(:,j) = [-y(j);x(j);z(j);1];%13,17,25
            
        elseif(j==8 || j==12 || j==21)
            pt(:,j) = [x(j);-z(j);y(j);1];%14,18,21
        elseif(j==9 || j==13 || j==22 || j==16)
            pt(:,j) = [z(j);-y(j);x(j);1];%15,19,22
        elseif(j==20)
            p = [x(j);y(j);z(j)];
            R = [cosd(-65.9746) -sind(-65.9746) 0;
                sind(-65.9746)  cosd(-65.9746)  0;
                0               0             1;];%20
            pt(:,j) = [R*p;1];
        elseif(j==23 || j==24 || j==18 || j==17)
            pt(:,j) = [-y(j);-z(j);x(j);1];%23,24
        else
            pt(:,j) = [y(j);z(j);x(j);1]; % 2,5,6,10,18,17,16
        end
        
        
    end
    
    
    cm.x(1) = y(1);
    cm.y(1) = -x(1);
    cm.z(1) = z(1);
    
    j=2;
    
    for i=1:8
        cm_pt = T_m(:,:,i)*pt(:,j);
        cm.x(j) = cm_pt(1);
        cm.y(j) = cm_pt(2);
        cm.z(j) = cm_pt(3);
        j = j+1;
    end
    
    for i=2:5
        cm_pt = T1_m(:,:,i)*pt(:,j);
        cm.x(j) = cm_pt(1);
        cm.y(j) = cm_pt(2);
        cm.z(j) = cm_pt(3);
        j = j+1;
    end
    
    for i=1:6
        cm_pt = T2_m(:,:,i)*pt(:,j);
        cm.x(j) = cm_pt(1);
        cm.y(j) = cm_pt(2);
        cm.z(j) = cm_pt(3);
        j = j+1;
    end
    
    for i=1:6
        cm_pt = T3_m(:,:,i)*pt(:,j);
        cm.x(j) = cm_pt(1);
        cm.y(j) = cm_pt(2);
        cm.z(j) = cm_pt(3);
        j = j+1;
    end
    com_final.x = sum(cm.x.*com.mass)/sum(com.mass);
    com_final.y = sum(cm.y.*cm.mass)/sum(cm.mass);
    com_final.z = sum(cm.z.*cm.mass)/sum(cm.mass);
    
    %     figure
    %     plot3(cm.x,cm.y,cm.z,'ko');
    %     hold on;
    %     plot3(cm.x,cm.y,cm.z,'b');
    %     plot3(com_final.x,com_final.y,com_final.z,'ro');
    %     axis equal;
    %     pause(0.05);
    %     hold off;
    
end
end
