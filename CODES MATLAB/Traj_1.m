function [x,y,z] = Traj_1()

%Trajectory
c = [160,23.7];
r = 10;
x = [];
y = [];
z = [];

for i=0:5:360
    x = [x,c(1)+r*cosd(i)];
    y = [y,0];
    z = [z,c(2)+r*sind(i)];
end

end

