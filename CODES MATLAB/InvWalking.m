 %# 3D inverted pendulum based walking implementation#


global robot;
global base;
global count;
global move_base;

prerun;

goal = [zeros(1,21);-COM_y(1,:)*1000;COM_x(1,:)*1000];
goal(1,:) = com.x;

g  = [0;-155;-8.6];

x1(1:21) = g(1);
y1(1:21) = g(2);
z1(1:21) = g(3);

Tr_mat = eye(4);

[f,q1,g,pts] = InvKin_com(goal(1,:),goal(2,:),goal(3,:),[x1;y1;z1],Tr_mat,27,1,theta,com);
theta = q1(:,21)';

%sample_draw;

if(count == 0)
    count = 1;
    move_base = 1;
end

com = ForwKin(theta,Tr_mat,zeros(1,28));

goal = [zeros(1,21);-COM_y(2,:)*1000;COM_x(2,:)*1000];
goal(1,:) = com.x;

[f,q2,g,pts1] = InvKin_com(goal(1,:),goal(2,:),goal(3,:),[x1;y1;z1],Tr_mat,27,1,zeros(1,28));
theta = q2(:,21)';

%sample_draw



if(count == 1)
count = 0;
end

Tr_mat = [1 0 0 0;
    0 1 0 0;
    0 0 1 142.1;
    0 0 0 1];

com = ForwKin(theta,Tr_mat,zeros(1,28));

goal = [zeros(1,21);-COM_y(3,:)*1000;COM_x(3,:)*1000];
goal(1,:) = com.x;
goal(3,:) = goal(3,:);

[x1,y1,z1]  = LegTraj(robot.parts(27).axis_loc,1,307.9,10,-5.1,20);

x1(21) = x1(20);
y1(21) = y1(20);
z1(21) = z1(20);

[f,q,g,pts2] = InvKin_com(goal(1,:),goal(2,:),goal(3,:),[x1;y1;z1],Tr_mat,27,1,zeros(1,28));

%sample_draw;

q_t = [q1,-q2,q];
x = [zeros(1,63)];
y = [-COM_y(1,:)*1000,-COM_y(2,:)*1000,-COM_y(3,:)*1000];
z = [COM_x(1,:)*1000,COM_x(2,:)*1000,COM_x(3,:)*1000]  ;

pp = [pts,pts1,pts2];

%follow_traj(x,y,z,q_t,1);
j=1;
for i = 1:63
    if(j==1)
        sample_draw(pts(:,:,i));
        if(i ==21)
            j=2;
        end
    elseif (j==2)
        sample_draw(pts1(:,:,i-21));
        if(i == 42)
            j=3;
        end
    else
        sample_draw(pts2(:,:,i-42));
    end
    F(i) = getframe;
end
%movie2avi(F,'Com_walk.avi');