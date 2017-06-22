clc;
clear all;
close all;


addpath('data');
addpath('path_planner\RRT');
% addpath('path_planner\RRT');

global robot;
global sim;
global base;
global move_base;
global count;
global steps;
global change;
global ph;

change = 1;
count=-1;
ph = 'ds';
move_base = 0;
base = 'rleg';
sim = 1; %0 for OFF and 1 for ON
robot = make_model();
if base == 'rleg'
    %Names and Ids for leg base
    r_arm = 16;
    l_arm = 21;
    l_leg = 28;
    from = 1;
    adj = [13,18];
    robot.parts(7).C_Id = [8,22];
    robot.parts(11).C_Id = [12,17];
elseif base == 'hips'
    %Names and Ids for hip base
    r_arm = 10;
    l_arm = 15;
    r_leg = 22;
    l_leg = 29;
    from = 23;
    adj = [7,12];
    robot.parts(1).C_Id = [2,16,23];
    robot.parts(5).C_Id = [6,11];
end

theta = zeros(1,28);
theta(adj(1)) = -5;
theta(adj(2)) = 5;

theta(3) = -10;
theta(2) = 10;

theta(25)= 10;
theta(26)= -10;

com = ForwKin(theta,eye(4),zeros(1,28));
ForwKin_Dyn(1);
% s_d
% goal = [com.x,com.y-100,com.z+100];
i = 1;
% Idx1 = find_route(20,1);
% J_tmp = zeros(6,28);
% 
% ll = length(Idx1);
% g1 = robot.parts(20).joint_loc;
% v = [0; 0; 0];

% while(1)
% err = goal - [com.x,com.y,com.z];
% % err1 = g1 - robot.parts(20).axis_loc;
% norm(err)
% robot.parts(20).joint_loc;
% % robot.parts(1).joint_loc;
% 
% if(norm(err) < 0.001)
%     break;
% end
% disp(i);
% J = Jacob_CoM ();
% % u1 = [robot.parts(Idx1).Z_w];
% % p1 = [robot.parts(Idx1).axis_loc];
% % J1 = Jacob(u1,p1);
% 
% % J_tmp(:,Idx1(1:ll-1)) = J1(:,:);
% % diff_vec = [com.x;com.y;com.z]-robot.parts(20).axis_loc;
% % op_mat = diff_vec*diff_vec';
% 
% J_g = eye(3)*(J);% - J_tmp(1:3,:) + op_mat*J_tmp(4:6,:));
% 
% %J_g = [J_g;J_tmp(1:3,:)];
% J_Inv = invsvd_lds(J_g,0.1);
% dq = J_Inv*[err'];
% theta = theta + rad2deg(dq)';
% 
% len = 28;
% 
% ang_limits = limits_2(base);
% theta_lim_min = ang_limits(1,:);
% theta_lim_max = ang_limits(2,:);    
% 
% for idx = 1:(len-1)
%     if(theta(idx)) < theta_lim_min(idx)
%         theta(idx) = theta_lim_min(idx);
%     elseif(theta(idx) > theta_lim_max(idx))
%         theta(idx) = theta_lim_max(idx);
%     end
% end
% 
% theta(7)=0;
% theta(10)=0;
% 
% % theta(20) = 0;
% % theta(21) = 0;
% % v1 = (-J_tmp(1:3,:))*dq;
% % v = v + v1;
% Tr_mat = eye(4);
% % Tr_mat(1:3,4) = v;
% % g1 = robot.parts(20).axis_loc + v;
% com = ForwKin(theta,Tr_mat,rad2deg(dq)');
% 
% i = i + 1;
% 
% end
%         
%% ------------------------------------------------------------------------

%%For Testing

% s1 = robot.parts(r_arm).joint_loc';
% s2 = robot.parts(l_arm).joint_loc';
% s3 = robot.parts(r_leg).joint_loc';
% s4 = robot.parts(l_leg).joint_loc';
%
% x1 = s1(1):0.1:s1(1)+3;
% x2 = s2(1):0.1:s2(1)+3;
% x3 = s3(1):0.1:s3(1)+3;
% x4 = s4(1):0.1:s4(1)+3;
%
% y1(1:31) = s1(2);
% y2(1:31) = s2(2);
% y3(1:31) = s3(2);
% y4(1:31) = s4(2);
%
% z1(1:31) = s1(3);
% z2(1:31) = s2(3);
% z3(1:31) = s3(3);
% z4(1:31) = s4(3);
%
%
% [flag,q] = InvKin_mult2([x1;x2;x3;x4],[y1;y2;y3;y4],[z1;z2;z3;z4],[r_arm,l_arm,r_leg,l_leg],[3,3,23,16],[]);
%
% follow_traj([x1,x2,x3,x4],[y1,y2,y3,y4],[z1,z2,z3,z4],[q],0);

%% ------------------------------------------------------------------------

%%For gait generation

% % [q,ang] = gen_gait();
% % figure
% % plot(q(:,2),'.-');
% % xlabel('ZMP_X');
% % ylabel('ZMP_Y');

%% ------------------------------------------------------------------------

%%For Walking


% steps = 3;
% 
% [x,z] = new_traj(2*steps-1);
% 
% xt = [];
% yt = [];
% zt = [];
% qt = [];
% vt = [];
% v = zeros(3,29);
% w = zeros(3,29);
% 
% for i = 1:3
%     if(i==1)
%         g1 = robot.parts(20).joint_loc';
%         g2 = robot.parts(27).joint_loc';
%         
%         x1(:,i) = (x(1:100)-94.1)+g1(1);
%         y1(:,i) = zeros(1,100)+g1(2);
%         z1(:,i) = g1(3)+z(1:100);
%         
%         gt1(1:100) = g2(1);
%         gt2(1:100) = g2(2);
%         gt3(1:100) = g2(3);
%         
%         for kk = 1:29
%             dd(:,:,kk) = robot.parts(kk).R_mat;
%         end
%         [flag,q{i},Dq,pos] = InvKin_mult(x1(:,i),y1(:,i),z1(:,i),20,gt1,gt2,gt3,27,18,25,adj,theta);
%         [dq,v1{i},w1{i}] = ForwVel2(deg2rad(q{i}),v,w);
%        % q{i}(24,100)= 0;
%        % break;
%        change = ~change;
%         
%     else
%         g1 = robot.parts(20).joint_loc';
%         g2 = robot.parts(27).joint_loc';
%         
%         x1(:,i) = (x(101:200)-94.1)+g1(1);
%         y1(:,i) = zeros(1,100)+g1(2);
%         z1(:,i) = g1(3) + z(101:200) - z(100);
%         
%         gt1(1:100) = g2(1);
%         gt2(1:100) = g2(2);
%         gt3(1:100) = g2(3);
%         theta = q1{i-1}(:,100);    
%         %theta(24)=0;
%         [flag,q{i},Dq,pos] = InvKin_mult(x1(:,i),y1(:,i),z1(:,i),20,gt1,gt2,gt3,27,18,25,adj,theta');
%         [dq,v1{i},w1{i}] = ForwVel2(deg2rad(q{i}),v2{i-1}(:,:,100),w2{i-1}(:,:,100));
%         change = ~change;
%     end
%     
%     
%     
%     g1 = robot.parts(20).joint_loc';
%     g2 = robot.parts(27).joint_loc';
%     
%     x2(:,i) = (x(101:200)-94.1)+g2(1);
%     y2(:,i) = zeros(1,100)+g2(2);
%     z2(:,i) = g2(3) + z(101:200) - z(100);
%     
%     gt1(1:100) = g1(1);
%     gt2(1:100) = g1(2);
%     gt3(1:100) = g1(3);
%     
%     theta = q{i}(:,100);
%    % theta(17)= 0;
%     [flag1,q1{i},Dq1,pos1] = InvKin_mult(gt1,gt2,gt3,20,x2(:,i),y2(:,i),z2(:,i),27,18,25,adj,theta');
%     [dq,v2{i},w2{i}] = ForwVel2(deg2rad(q1{i}),v1{i}(:,:,100),w1{i}(:,:,100));
%     change = ~change;
%     
%     xt = [xt;x1(:,i);x2(:,i)];
%     yt = [yt;y1(:,i);y2(:,i)];
%     zt = [zt;z1(:,i);z2(:,i)];
%     qt = [qt,q{i},q1{i}];
%     vt = [vt,v1{i},v2{i}];
% end
% figure(2),plot([100:600],qt(18,100:600),'r.-',[100:600],qt(19,100:600),'b.-'); 
% figure(3),plot([100:600],qt(25,100:600),'r.-',[100:600],qt(26,100:600),'b.-',[100:600],qt(27,100:600),'k.-',[100:600],qt(23,100:600),'y.-',[100:600],qt(24,100:600),'c.-');
% legend('hip3','knee','ankle','hip1','hip2');
% [tm1,tm2,tm3] = follow_traj(xt,yt,zt,qt,1);
% dp = diff(tm1,1,2)
% dl = diff(tm2,1,2)
% pz = (4000*9800*tm3(3,1:599) - dl(2,:))./(4000*9800+dp(1,:))
% py = (4000*9800*tm3(2,1:599) + dl(3,:))./(4000*9800+dp(1,:))
% figure,plot(pz,'.-');
% figure,plot(py,'.-')
% ------------------------------------------------------------------------

%For Testinh RRT

% start = robot.parts(l_arm).joint_loc';
% start(1) = start(1)+50;
% start(2) = start(2);
% start(3) = start(3);
% goal = start;
% goal(2) = goal(2)-70;
% centre = (start + goal)/2;
% r=10;
% 
% [traj,q] = rrt_connect(start,goal,l_arm,from,adj);
% 
% start = robot.parts(l_arm).joint_loc'
% start(1) = start(1);
% start(2) = start(2);
% start(3) = start(3);
% goal = start;
% goal(3) = goal(3)+30;
% 
% [traj1,q1] = rrt_connect(start,goal,l_arm,1,adj);
% 
% x = [traj(1,:),traj1(1,:)];
% y = [traj(2,:),traj1(2,:)];
% z = [traj(3,:),traj1(3,:)];

%% ------------------------------------------------------------------------

%For Drawing in zero theta
% 
%  p = [robot.draw];
%  if(base == 'hips')
% %%For Hip
% X = p(1,1:10);
% Y = p(2,1:10);
% Z = p(3,1:10);
% 
% X1 = [p(1,5), p(1,11:15)];
% Y1 = [p(2,5), p(2,11:15)];
% Z1 = [p(3,5), p(3,11:15)];
% 
% X2 = [p(1,1), p(1,16:22)];
% Y2 = [p(2,1), p(2,16:22)];
% Z2 = [p(3,1), p(3,16:22)];
% 
% X3 = [p(1,1), p(1,23:29)];
% Y3 = [p(2,1), p(2,23:29)];
% Z3 = [p(3,1), p(3,23:29)];
% 
% figure(1)
% plot3(X,Y,Z,'k.-');
% hold on;
% plot3(X,Y,Z,'bo');
% plot3(X1,Y1,Z1,'k.-');
% plot3(X1,Y1,Z1,'bo');
% plot3(X2,Y2,Z2,'k.-');
% plot3(X2,Y2,Z2,'ro');
% plot3(X3,Y3,Z3,'k.-');
% plot3(X3,Y3,Z3,'co');
% axis equal
% 
%  else
% %For Leg
% X = p(1,1:17);
% Y = p(2,1:17);
% Z = p(3,1:17);
% 
% X1 = [p(1,12), p(1,18:22)];
% Y1 = [p(2,12), p(2,18:22)];
% Z1 = [p(3,12), p(3,18:22)];
% % 
% X2 = [p(1,8),p(1,23:29)];
% Y2 = [p(2,8),p(2,23:29)];
% Z2 = [p(3,8),p(3,23:29)];
% 
% [xs,ys,zs] = sphere;
% 
% figure(1)
% plot3(X,Y,Z,'k.-');
% hold on;
% plot3(X,Y,Z,'bo');
% plot3(X1,Y1,Z1,'k.-');
% plot3(X1,Y1,Z1,'bo');
% plot3(X2,Y2,Z2,'k.-');
% plot3(X2,Y2,Z2,'bo');
% axis equal
%  end
%%-------------------------------------------------------------------------

%% For Manipulation

% start = robot.parts(r_arm).joint_loc';
% goal = start;
% goal(3) = goal(3) + 100;
% goal(1) = goal(1) - 20;
% goal(2) = goal(2);
% 
% [traj1,q] = rrt_connect(start,goal,r_arm,8,adj);
% 
% x1 = traj1(1,:);
% y1 = traj1(2,:);
% z1 = traj1(3,:);
% 
% start = robot.parts(l_arm).joint_loc';
% goal = start;
% 
% goal(3) = goal(3) + 100;
% goal(1) = goal(1) - 20;
% goal(2) = goal(2);
% 
% 
% [traj2,q1] = rrt_connect(start,goal,l_arm,8,adj);
% 
% x2 = traj2(1,:);
% y2 = traj2(2,:);
% z2 = traj2(3,:);
% 
% [flag,ang,Dq,pos] = InvKin_mult(x1,y1,z1,r_arm,x2,y2,z2,l_arm,8,8,adj,theta,zeros(1,28));
% theta = ang(:,end)';
% dq = Dq(:,end)';
% robot = make_model();
% if base == 'rleg'
%     %Names and Ids for leg base
%     r_arm = 16;
%     l_arm = 21;
%     l_leg = 28;
%     from = 1;
%     adj = [13,18];
%     robot.parts(7).C_Id = [8,22];
%     robot.parts(11).C_Id = [12,17];
% elseif base == 'hips'
%     %Names and Ids for hip base
%     r_arm = 10;
%     l_arm = 15;
%     r_leg = 22;
%     l_leg = 29;
%     from = 23;
%     adj = [7,12];
%     robot.parts(1).C_Id = [2,16,23];
%     robot.parts(5).C_Id = [6,11];
% end
% com = ForwKin(theta,eye(4),zeros(1,28));
% ForwKin_Dyn(1);
% pause
% [flag,ang,Dq,pos] = InvKin_mult(x1(end:-1:1),y1(end:-1:1),z1(end:-1:1),r_arm,x2(end:-1:1),y2(end:-1:1),z2(end:-1:1),l_arm,8,8,adj,theta,dq);
%% ---------------------------------------------------------------------------------
%% For Proof
% g1 = robot.parts(l_arm).joint_loc';
% 
% x = g1(1)+linspace(20,50,20).*sin([1:20]*pi/40);
% y = g1(2)+zeros(1,20);
% z = g1(3)+linspace(1,70,20);
% 
% [flag,q,Dq,pos] = InvKin(x,y,z,l_arm,8,adj,theta);

% g1 = robot.parts(l_arm).joint_loc';
% 
% x = g1(1)+zeros(1,20);
% y = g1(2)+linspace(1,100,20); 
% z = g1(3)+30+zeros(1,20);
% 
% [flag,q,Dq,pos] = InvKin(x,y,z,l_arm,8,adj,theta);
% 
% follow_traj([x],[y],[z],[q],1);

%% ---------------------------------------------------------------------------------------
%%For RL

start = robot.parts(r_arm).joint_loc';
goal = start;
goal(3) = goal(3) + 100;
goal(1) = goal(1) - 50;
goal(2) = goal(2);


[Q,Pol] = QAC(start,goal,1000);

% [traj1,q] = rrt_connect(start,goal,r_arm,8,adj);

% x1 = traj1(1,:);
% y1 = traj1(2,:);
% z1 = traj1(3,:);
%  
% [flag,q,Dq,pos,cdl,hG] = InvKin(x1,y1,z1,r_arm,7,adj,theta);
