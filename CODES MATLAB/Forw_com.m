clear;
clc;
addpath('C:\Users\Phani\Dropbox\DH\RobotModel\Matlab\maincode\data');
addpath('C:\Users\Phani\Dropbox\DH\RobotModel\Matlab\maincode\path_planner\RRT-connectstart');
global robot;
global sim;
global base;
global move_base;
global count;
global change;
change = 1;
count=-1;
move_base = 0;
base = 'leg';
sim = 1; %0 for OFF and 1 for ON
robot = make_model();
if base == 'leg'
    %Names and Ids for leg base
    r_arm = 16;
    l_arm = 21;
    l_leg = 28;
    from = 1;
    adj = [13,18];
elseif base == 'hip'
    %Names and Ids for hip base
    r_arm = 10;
    l_arm = 15;
    r_leg = 22;
    l_leg = 29;
    from = 23;
    adj = [7,12];
end

x = sin([1:100].*2*pi/100);

theta = zeros(1,28);
%theta(4) = 30;

[com,cm] = ForwKin(theta,eye(4),zeros(1,28));

T = [0 0 -1 0;0 1 0 0; 1 0 0 58.1;0 0 0 1];
T = eye(4);
c = T*[com.x;com.y;com.z;1];

p = [robot.draw];

if base == 'hip'
    %T = [0 0 -1 0;0 1 0 0; 1 0 0 483.898;0 0 0 1];
    c = T*[com.x;com.y;com.z;1];
    % %%For Hip
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
    
    for kk=1:25
                    ha = [cm.x(kk);cm.y(kk);cm.z(kk);1];
                    ha = T*ha;
                    cm.x(kk) = ha(1);
                    cm.y(kk) = ha(2);
                    cm.z(kk) = ha(3);
                end
    
    figure(1)
    cm.x
    cm.y
    cm.z
    
    plot3(X,Y,Z,'k.-');
    hold on;
    plot3(X,Y,Z,'bo');
    plot3(X1,Y1,Z1,'k.-');
    plot3(X1,Y1,Z1,'bo');
    plot3(X2,Y2,Z2,'k.-');
    plot3(X2,Y2,Z2,'ro');
    plot3(X3,Y3,Z3,'k.-');
    plot3(X3,Y3,Z3,'bo');
    plot3(cm.x(1:15),cm.y(1:15),cm.z(1:15),'co');
    plot3(c(1),c(2),c(3),'ro');
    axis equal
    
else
    %For Leg
    X = p(1,1:17);
    Y = p(2,1:17);
    Z = p(3,1:17);
    
    X1 = [p(1,12), p(1,18:22)];
    Y1 = [p(2,12), p(2,18:22)];
    Z1 = [p(3,12), p(3,18:22)];
    
    X2 = [p(1,8),p(1,23:29)];
    Y2 = [p(2,8),p(2,23:29)];
    Z2 = [p(3,8),p(3,23:29)];
    
    [xs,ys,zs] = sphere;
        for kk=1:25
                    ha = [cm.x(kk);cm.y(kk);cm.z(kk);1];
                    ha = T*ha;
                    cm.x(kk) = ha(1);
                    cm.y(kk) = ha(2);
                    cm.z(kk) = ha(3);
                end
    figure(2)
    c
    cm.x
    cm.y
    cm.z
    
    plot3(X,Y,Z,'k.-');
    hold on;
    plot3(X,Y,Z,'bo');
    plot3(X1,Y1,Z1,'k.-');
    plot3(X1,Y1,Z1,'bo');
    plot3(X2,Y2,Z2,'k.-');
    plot3(X2,Y2,Z2,'bo');
    plot3(cm.x(1:25),cm.y(1:25),cm.z(1:25),'co');
    plot3(c(1),c(2),c(3),'ro');
    axis equal;
end