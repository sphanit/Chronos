function [traj] = rrt(start,goal)
path.x(1) = start(1);
path.y(1) = start(2);
path.z(1) = start(3);
path.parent(1)=1;

gx=goal(1);
gy=goal(2);
gz=goal(3);

delta = 3;

i=2;
while(1)
    q_rand = qrand(goal);
    np = nn(q_rand,path);
    q_near = [path.x(np),path.y(np),path.z(np)];
    q_new = move(q_near,q_rand,delta);
    
    path.x(i) = q_new(1);
    path.y(i) = q_new(2);
    path.z(i) = q_new(3);
    path.parent(i) = np;
    
     d=sqrt((path.x(i)-gx)^2+(path.y(i)-gy)^2+(path.z(i)-gz)^2);
     
     if(abs(d)<=3)
        break;
     end
    
     i=i+1;
    
end

final = i;
pt = [path.x(final);path.y(final);path.z(final)];
traj = pt;

while(1)
    j=path.parent(final);
    pt = [path.x(j);path.y(j);path.z(j)];

    if(j==1)
        break;
    end
    
    traj=[pt,traj];
    
    final=j;
    
end

traj = [start',traj,goal'];

end

