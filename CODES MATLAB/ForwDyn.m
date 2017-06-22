function [] = ForwDyn()
global robot;
nDof = 28 + 6;
A = zeros(nDof,nDof);
b = InvDyn_cal(0);

for n = 1:nDof
    A(:,n) = InvDyn_cal(n)-b;
end

%Motor Inertia???
u = [0 0 0 0 0 0 robot.parts(1:28).u]';
ddq = (A^-1)*(-b + u);

robot.parts(1).dv0 = ddq(1:3);
robot.parts(1).dw = ddq(4:6);

for j = 2:28
robot.parts(j).ddq = ddq(j+6);
end
end