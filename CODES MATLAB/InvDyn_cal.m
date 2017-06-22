function [ ret ] = InvDyn_cal(n)
global robot;

robot.parts(1).dv0 = [0;0;0];
robot.parts(1).dw0 = [0;0;0];

if(n>=1 & n<=3)
    robot.parts(1).dv0(n) = 1;
elseif(n>=4 & n<=6)
    robot.parts(1).dw(n-3) = 1;
end

for i=1:28
    if(i == n-6)
        robot.parts(i).ddq = 1;
    else
        robot.parts(i).ddq = 0;
    end
end
ForwKin_Dyn(1);
[f,tau] = InvDyn(1);
ret = [f',tau',robot.parts(1:28).u]';

end

