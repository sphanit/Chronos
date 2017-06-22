function [ZMP,q] = gen_gait()
global robot;
global base;
global move_base;

move_base = 1;

q = zeros(28,100);

T = 0.3;
time = linspace(0.000001,0.4,100);

%accx = 0*((sign(time)-sign(time-0.1))/2) +((800/27).*(time-0.1).*((sign(time-0.1)-sign(time-0.25))/2) + ((40/9)-((800/27).*(time-0.25))).*((sign(time-0.25)+1)/2)).*((sign(time-0.1)+1)/2);
%accy = 0;
x = 0*((sign(time)-sign(time-0.1))/2) + (1.3333)*((sign(time-0.1)+1)/2);
%% Left Leg
%knee
l_knee = (-0.01)*((sign(time)-sign(time-0.1))/2) +((-(pi/6)*(1-cos(2*pi*(time-0.1)/0.3))-0.01).*((sign(time-0.1)-sign(time-0.25))/2) + (-pi/6 - (pi/12)*(1-cos(2*pi*(time-0.1)/0.3))-0.01).*((sign(time-0.25)+1)/2)).*((sign(time-0.1)+1)/2);
% figure(2);
% plot(time,l_knee);
% xlabel('time');
% ylabel('angle in radian');
%hip_1
l_hip1 = (-0.3604)*((sign(time)-sign(time-0.1))/2) + ((-0.1802*(1+cos(2*pi*(time-0.1)/0.3)) + (pi/12)*(1-cos(2*pi*(time-0.1)/0.3))).*((sign(time-0.1)-sign(time-0.25))/2) + ( 0.1253*(1+cos(2*pi*(time-0.1)/0.3)) + (pi/8) - (pi/24)*cos(2*pi*(time-0.1)/0.3) ).*((sign(time-0.25)+1)/2)).*((sign(time-0.1)+1)/2);
% figure(2);
% hold on;
% plot(time,l_hip1);
% hold off;

%hip_2
l_hip2 = 0*((sign(time)-sign(time-0.1))/2)+(-(pi/24)*sin(pi*(time-0.1)/0.3)).*((sign(time-0.1)+1)/2);
% figure(2);
% hold on;
% plot(time,l_hip2);
% hold off;

%ankle
l_ankle = 0.39*((sign(time)-sign(time-0.1))/2)+ (0.39+time-0.1).*((sign(time-0.1)+1)/2);
% figure(2);
% hold on;
% plot(time,l_ankle);
% hold off;
% legend('knee','hip1','hip2','ankle');
%% Right Leg
%knee
r_knee = (-pi/6)*((sign(time)-sign(time-0.1))/2)+(-(pi/6).*((sign(time-0.1)-sign(time-0.25))/2) -((pi/12)*(1-cos(2*pi*(time-0.1)/0.3))).*((sign(time-0.25)+1)/2)).*((sign(time-0.1)+1)/2);
rad2deg(r_knee)
% figure(3);
% plot(time,r_knee);
% xlabel('time');
% ylabel('angle in radian');

%hip_1
r_hip1 = 0.5124*((sign(time)-sign(time-0.1))/2) + ((0.1253*(1+cos(2*pi*(time-0.1)/0.3)) + (pi/12)).*((sign(time-0.1)-sign(time-0.25))/2) + (-0.1802*(1+cos(2*pi*(time-0.1)/0.3)) + (pi/24)*(1-cos(2*pi*(time-0.1)/0.3))).*((sign(time-0.25)+1)/2)).*((sign(time-0.1)+1)/2);
% figure(3);
% hold on;
% plot(time,r_hip1);
% hold off;

%hip_2
r_hip2 = 0*((sign(time)-sign(time-0.1))/2)+(-(pi/24)*sin(pi*(time-0.1)/0.3)).*((sign(time-0.1)+1)/2);
% figure(3);
% hold on;
% plot(time,r_hip2);
% hold off;
%ankle
r_ankle = 0.39*((sign(time)-sign(time-0.1))/2)+ (0.39+(0.35/0.3)*(time-0.1)).*((sign(time-0.1)+1)/2);
% figure(3);
% hold on;
% plot(time,r_ankle);
% hold off;

%ankle_2
r_ankle2 = 0*((sign(time)-sign(time-0.1))/2)+(-1.5*(time-0.1)).*((sign(time-0.1)+1)/2);

% figure(3);
% hold on;
% plot(time,r_ankle2);
% hold off;
% legend('knee','hip1','hip2','ankle','ankle2');

%% Abdomen
ab1 = 0*((sign(time)-sign(time-0.1))/2) + (-(pi/24)*sin(pi*(time-0.1)/0.3)).*((sign(time-0.1)+1)/2);
% figure(4);
% plot(time,ab1);
% xlabel('time');
% ylabel('angle in radian');
% title('abdomen_1');


%%

if(base == 'hips')
    q(27,:) = -rad2deg(l_ankle);
    q(26,:) = -rad2deg(l_knee)
    q(25,:) = -rad2deg(l_hip1);
    q(23,:) = -rad2deg(l_hip2);
    
    q(21,:) = -rad2deg(r_ankle2);
    q(20,:) = -rad2deg(r_ankle);
    q(19,:) = -rad2deg(r_knee);
    q(18,:) = -rad2deg(r_hip1);
    q(16,:) = -rad2deg(r_hip2);
    q(1,:) = -rad2deg(ab1);
    T = [0 0 -1 0;0 1 0 0; 1 0 0 483.898;0 0 0 1];
    lq = size(q);
    lq = lq(2)
    x_s = 0;
    dq = diff(q,1,2);
    dq = [dq,zeros(28,1)];
    P_pre = 0;
    L_pre = 0;
    for i=1:lq
        x_s = x_s + x(i)
        Tr_mat = [1 0 0 0;
            0 1 0 0;
            0 0 1 -x_s;
            0 0 0 1];
        [com,c,P,L] = ForwKin(q(:,i)',Tr_mat,dq(:,i));
        cm = T*[com.x;com.y;com.z;1];
        P = T*[P(1);P(2);P(3);1];
        L = T*[L(1);L(2);L(3);1];
        
        for kk=1:25
            ha = [c.x(kk);c.y(kk);c.z(kk);1];
            ha = T*ha;
            c.x(kk) = ha(1);
            c.y(kk) = ha(2);
            c.z(kk) = ha(3);
        end
        com.x = cm(1);
        com.y = cm(2);
        com.z = cm(3);
        
        %         x_zmp = com.x - accx(i)*com.z/9.8;
        %         y_zmp = com.y - accy*com.z/9.8;
        %
        
        dP = P-P_pre;
        dL = L-L_pre;
        px = (4500*9800*com.x - dL(2,:))/(4500*9800+dP(3,:));
        py = (4500*9800*com.y + dL(1,:))/(4500*9800+dP(3,:));
        ZMP(i,:) = [px;py];
        
        
        p = [robot.draw];
        
        
        p = [robot.draw];
        for kk = 1:29
            tmp = T*[p(1,kk);p(2,kk);p(3,kk);1];
            p(:,kk) = tmp(1:3);
        end
        foot1_x = [p(1,29)-60,p(1,29)+90,p(1,29)+90,p(1,29)-60,p(1,29)-60];
        foot1_y = [p(2,29)+23,p(2,29)+23,p(2,29)-23,p(2,29)-23,p(2,29)+23];
        foot1_z = [p(3,29),p(3,29),p(3,29),p(3,29),p(3,29)];
        
        foot2_x = [p(1,22)-60,p(1,22)+90,p(1,22)+90,p(1,22)-60,p(1,22)-60];
        foot2_y = [p(2,22)+23,p(2,22)+23,p(2,22)-23,p(2,22)-23,p(2,22)+23];
        foot2_z = [p(3,22),p(3,22),p(3,22),p(3,22),p(3,22)];
        
        X = p(1,1:10);
        Y = p(2,1:10);
        Z = p(3,1:10);
        
        X1 = [p(1,5), p(1,11:15)];
        Y1 = [p(2,5), p(2,11:15)];
        Z1 = [p(3,5), p(3,11:15)];
        
        X2 = [p(1,1), p(1,16:22)];
        Y2 = [p(2,1), p(2,16:22)];
        Z2 = [p(3,1), p(3,16:22)];
        
        X3 = [p(1,1), p(1,23:29)];
        Y3 = [p(2,1), p(2,23:29)];
        Z3 = [p(3,1), p(3,23:29)];
        
        figure(1)
        plot3(X,Y,Z,'k.-');
        hold on;
        plot3(X,Y,Z,'bo');
        plot3(X1,Y1,Z1,'k.-');
        plot3(X1,Y1,Z1,'bo');
        plot3(X2,Y2,Z2,'k.-');
        plot3(X2,Y2,Z2,'bo');
        plot3(X3,Y3,Z3,'k.-');
        plot3(X3,Y3,Z3,'bo');
        plot3(com.x,com.y,com.z,'ro');
        %plot3(com.x,com.y,0,'ro');
        plot3(px,py,foot2_z,'go')
        plot3(c.x(1:25),c.y(1:25),c.z(1:25),'co');
        plot3(foot1_x,foot1_y,foot1_z,'y');
        plot3(foot2_x,foot2_y,foot2_z,'y');
        hold off;
        axis([-300 300 -300 300 -450 350])
        axis equal;
        xlabel('x')
        ylabel('y')
        zlabel('z')
        grid on
        pause(0.1)
        P_pre = P;
        L_pre = L;
    end
else
    %     q(26,:) = rad2deg(l_ankle);
    %     q(25,:) = rad2deg(l_knee);
    %     q(24,:) = rad2deg(l_hip1);
    %     q(22,:) = rad2deg(l_hip2);
    %
    %     q(1,:) = rad2deg(r_ankle2);
    %     q(2,:) = rad2deg(r_ankle);
    %     q(3,:) = rad2deg(r_knee);
    %     q(4,:) = rad2deg(r_hip1);
    %     q(6,:) = rad2deg(r_hip2);
    q(7,:) = rad2deg(ab1);
    T = [0 0 -1 0;0 1 0 0; 1 0 0 58.1;0 0 0 1];
    lq = size(q);
    lq = lq(2);
    for i=1:lq
        [com,c,P,L] = ForwKin(q(:,i)',eye(4),zeros(1,28));
        cm = T*[com.x;com.y;com.z;1];
        for kk=1:25
            ha = [c.x(kk);c.y(kk);c.z(kk);1];
            ha = T*ha;
            c.x(kk) = ha(1);
            c.y(kk) = ha(2);
            c.z(kk) = ha(3);
        end
        com.x = cm(1);
        com.y = cm(2);
        com.z = cm(3);
        CoM(i,:) = [com.x;com.y;com.z];
        
        p = [robot.draw];
        
        ret(i,:) = p(:,16);
        
        X = p(1,1:17);
        Y = p(2,1:17);
        Z = p(3,1:17);
        
        X1 = [p(1,12), p(1,18:22)];
        Y1 = [p(2,12), p(2,18:22)];
        Z1 = [p(3,12), p(3,18:22)];
        
        X2 = [p(1,8),p(1,23:29)];
        Y2 = [p(2,8),p(2,23:29)];
        Z2 = [p(3,8),p(3,23:29)];
        
        
        figure(1)
        plot3(X,Y,Z,'k.-');
        hold on;
        plot3(X,Y,Z,'bo');
        plot3(X1,Y1,Z1,'k.-');
        plot3(X1,Y1,Z1,'bo');
        plot3(X2,Y2,Z2,'k.-');
        plot3(X2,Y2,Z2,'bo');
        plot3(com.x,com.y,com.z,'ro');
        plot3(com.x,com.y,0,'go');
        plot3(c.x,c.y,c.z,'co');
        hold off;
        axis([-300 300 -300 300 0 800])
        grid on
        axis equal;
        pause(0.001)
    end
end
end
