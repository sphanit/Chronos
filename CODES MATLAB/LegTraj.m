function [x1,y1,z1]  = LegTraj(l_pos,orie,sLen, sHt, sDisp, itr)
% arguments are leg pos(1x3), orientaion(3x3), length, height, displacement
% in y, no i

[z1, x1] = bezier1(sHt, sLen, itr);
y1 = zeros(itr,1)' + l_pos(2);

% l_pos = robot.parts(l_leg).joint_loc'

x1 = l_pos(1) + x1;

% y1 = l_pos(2) + y1;
z1 = l_pos(3) + z1;

for i = 1:itr
    y1(i) = (sDisp/itr)*i + y1(i);

end

end

    
    
    
% [flag,q,Dq,pos] = InvKin(x1,y1,z1,l_leg,from,adj);
% 
% follow_traj(x1,y1,z1,q,0);
