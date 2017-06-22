function [flag] = collision_check()
global robot;
flag = 1;

list1 = [1:8,10,13:22];
list2 = [1:10,14,17:22];
list3 = [1:5,18:22];
list4 = [1,2,3];

for j=1:length(list1)
    i = list1(j);
    p1 = 11;
    p2 = 12;
    r1 = robot.link(p1).radius;
    r2 = robot.link(p2).radius;
    r = robot.link(i).radius;
    
    if(robot.link(i).type == 'c')
        d1 = line_to_line(robot.link(i).pts(:,1),robot.link(i).pts(:,2),robot.link(p1).pts(:,1),robot.link(p1).pts(:,2));
        d2 = line_to_line(robot.link(i).pts(:,1),robot.link(i).pts(:,2),robot.link(p2).pts(:,1),robot.link(p2).pts(:,2));
    else
        d1 = point_to_line(robot.link(i).pts(:,1),robot.link(p1).pts(:,1),robot.link(p1).pts(:,2));
        d2 = point_to_line(robot.link(i).pts(:,1),robot.link(p2).pts(:,1),robot.link(p2).pts(:,2));
    end
    
    if(i ~= 10)
        if(d1 < r+r1 || d2 < r+r2)
            fprintf('collison with %s \n', robot.link(i).name);
            flag = 0;
        end
    else
        if(d2 < r+r2)
            flag = 0;
        end
    end
end

for j=1:length(list2)
    i = list2(j);
    p1 = 15;
    p2 = 16;
    r1 = robot.link(p1).radius;
    r2 = robot.link(p2).radius;
    r = robot.link(i).radius;
    
    if(robot.link(i).type == 'c')
        d1 = line_to_line(robot.link(i).pts(:,1),robot.link(i).pts(:,2),robot.link(p1).pts(:,1),robot.link(p1).pts(:,2));
        d2 = line_to_line(robot.link(i).pts(:,1),robot.link(i).pts(:,2),robot.link(p2).pts(:,1),robot.link(p2).pts(:,2));
    else
        d1 = point_to_line(robot.link(i).pts(:,1),robot.link(p1).pts(:,1),robot.link(p1).pts(:,2));
        d2 = point_to_line(robot.link(i).pts(:,1),robot.link(p2).pts(:,1),robot.link(p2).pts(:,2));
    end
    
    if(i ~= 14)
        if(d1 < r+r1 || d2 < r+r2)
            fprintf('collison with %s \n', robot.link(i).name);
            flag = 0;
        end
    else
        if(d2 < r+r2)
            flag = 0;
        end
    end
end

for j=1:length(list3)
    i = list3(j);
    p1 = 8;
    p2 = 9;
    p3 = 10;
    p4 = 13;
    p5 = 14;
    r1 = robot.link(p1).radius;
    r2 = robot.link(p2).radius;
    r3 = robot.link(p3).radius;
    r4 = robot.link(p4).radius;
    r5 = robot.link(p5).radius;
    r = robot.link(i).radius;
    
    if(robot.link(i).type == 'c')
        d1 = line_to_line(robot.link(i).pts(:,1),robot.link(i).pts(:,2),robot.link(p1).pts(:,1),robot.link(p1).pts(:,2));
        d2 = line_to_line(robot.link(i).pts(:,1),robot.link(i).pts(:,2),robot.link(p2).pts(:,1),robot.link(p2).pts(:,2));
        d3 = point_to_line(robot.link(p3).pts(:,1),robot.link(i).pts(:,1),robot.link(i).pts(:,2));
        d4 = line_to_line(robot.link(i).pts(:,1),robot.link(i).pts(:,2),robot.link(p4).pts(:,1),robot.link(p2).pts(:,2));
        d5 = point_to_line(robot.link(p5).pts(:,1),robot.link(i).pts(:,1),robot.link(i).pts(:,2));
    else
        d1 = point_to_line(robot.link(i).pts(:,1),robot.link(p1).pts(:,1),robot.link(p1).pts(:,2));
        d2 = point_to_line(robot.link(i).pts(:,1),robot.link(p2).pts(:,1),robot.link(p2).pts(:,2));
        d3 = norm(robot.link(i).pts(:,1)-robot.link(p3).pts(:,1));
        d4 = point_to_line(robot.link(i).pts(:,1),robot.link(p4).pts(:,1),robot.link(p4).pts(:,2));
        d5 = norm(robot.link(i).pts(:,1)-robot.link(p5).pts(:,1));
    end
    
    
    if(d1 < r+r1 || d2 < r+r2 || d3 < r+r3 || d4 < r+r4 || d5 < r+r5)
        fprintf('collison with %s \n', robot.link(i).name);
        flag = 0;
    end
    
end

for j=1:length(list4)
    i = list3(j);
    p1 = 20;
    p2 = 21;
    p3 = 22;
    r1 = robot.link(p1).radius;
    r2 = robot.link(p2).radius;
    r3 = robot.link(p3).radius;
    r = robot.link(i).radius;
    
    if(robot.link(i).type == 'c')
        d1 = line_to_line(robot.link(i).pts(:,1),robot.link(i).pts(:,2),robot.link(p1).pts(:,1),robot.link(p1).pts(:,2));
        d2 = line_to_line(robot.link(i).pts(:,1),robot.link(i).pts(:,2),robot.link(p2).pts(:,1),robot.link(p2).pts(:,2));
        d3 = point_to_line(robot.link(p3).pts(:,1),robot.link(i).pts(:,1),robot.link(i).pts(:,2));
    else
        d1 = point_to_line(robot.link(i).pts(:,1),robot.link(p1).pts(:,1),robot.link(p1).pts(:,2));
        d2 = point_to_line(robot.link(i).pts(:,1),robot.link(p2).pts(:,1),robot.link(p2).pts(:,2));
        d3 = norm(robot.link(i).pts(:,1)-robot.link(p3).pts(:,1));
    end
    
    
    if(d1 < r+r1 || d2 < r+r2 || d3 < r+r3)
       fprintf('collison with %s \n', robot.link(i).name);
        flag = 0;
    end
    
end


end