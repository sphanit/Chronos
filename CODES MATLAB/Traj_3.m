function [x,y,z] = Traj_3()

%Trajectory3
c = [400,60,0];
r = 30;
x = [];
y = [];
z = [];
for i=0:5:360
    z = [z,c(3)+r*sind(i)];
    y = [y,c(2)+r*cosd(i)];
    x = [x,abs(sqrt(c(1)^2-r^2))];
end

end