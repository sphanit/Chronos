function [Idx] = find_route(part_Id,from)
global robot
if(part_Id==from)
    Idx = [part_Id];
else
    Idx = [find_route(robot.parts(part_Id).P_Id,from) part_Id];
end

