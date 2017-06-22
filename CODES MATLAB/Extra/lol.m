function [f] = lol(j)
global robot
if(j == 0)
    f = 0;
    return;
end

f = 1;
disp(j);
robot.parts(j).add = j;
if(length(robot.parts(j).C_Id) == 2)
    disp('enter1')
    f1 = lol(robot.parts(j).C_Id(1));
    f = f + f1;
    disp('enter2')
    f2 = lol(robot.parts(j).C_Id(2));
    f = f + f2;
else
    f1 = lol(robot.parts(j).C_Id);
    f = f + f1;
end

end

