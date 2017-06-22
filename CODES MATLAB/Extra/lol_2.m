function [f] = lol_2(j)
global robot
global temp
if(j == 0)
    f = 0;
    return;
end

f = 1;


if(length(robot.parts(j).C_Id) == 2)
    f1 = lol_2(robot.parts(j).C_Id(1));
    f = f + f1;
    if(j ~=7)
        f2 = lol_2(robot.parts(j).C_Id(2));
        f = f + f2;
    else
        lol_2(robot.parts(j).C_Id(2));
    end
else
    if(j<=21)
        f1 = lol_2(robot.parts(j).C_Id);
        f = f + f1;
    else
        f = temp + f;
        temp = f;
        lol_2(robot.parts(j).C_Id);
    end
end
if (j == 7)
%     disp('f = ')
%     disp(f);
    f = f/2;
    temp = f;
end

% if(j==28)
%     disp('ff = ')
%     disp(f)
% end
j
f

end

