function[] = s_d(cm,com)
%For Drawing in zero theta
global count
global base
global robot
p = [robot.draw];
T = [0 0 -1 0;0 1 0 0; 1 0 0 58.1;0 0 0 1];
if(base == 'hips')
    %%For Hip
    X = p(1,1:10);
    Y = p(2,1:10);
    Z = p(3,1:10);
    
    X1 = [p(1,5), p(1,11:15)];
    Y1 = [p(2,5), p(2,11:15)];
    Z1 = [p(3,5), p(3,11:15)];
    
    X2 = [p(1,1), p(1,16:22)];
    Y2 = [p(2,1), p(2,16:22)];
    Z2 = [p(3,1), p(3,16:22)];
    
    X3 = [p(1,1), p(1,23:29)];
    Y3 = [p(2,1), p(2,23:29)];
    Z3 = [p(3,1), p(3,23:29)];
    
    if(count == 0)
        figure(1)
        plot3(X,Y,Z,'k.-');
        hold on;
        plot3(X,Y,Z,'bo');
        plot3(X1,Y1,Z1,'k.-');
        plot3(X1,Y1,Z1,'bo');
        plot3(X2,Y2,Z2,'k.-');
        plot3(X2,Y2,Z2,'ro');
        plot3(X3,Y3,Z3,'k.-');
        plot3(X3,Y3,Z3,'co');
        plot3(cm.x,cm.y,cm.z,'co');
        plot3(com.x,com.y,com.z,'ro');
        axis equal
    else
        figure(1)
        plot3(X,Y,Z,'r.-');
        hold on;
        plot3(X,Y,Z,'bo');
        plot3(X1,Y1,Z1,'r.-');
        plot3(X1,Y1,Z1,'bo');
        plot3(X2,Y2,Z2,'r.-');
        plot3(X2,Y2,Z2,'ro');
        plot3(X3,Y3,Z3,'r.-');
        plot3(X3,Y3,Z3,'co');
        plot3(cm.x,cm.y,cm.z,'co');
        plot3(com.x,com.y,com.z,'ro');
        axis equal
        
    end
else
    %For Leg
    
    for i=1:29
        tmp = p(:,i);
        tmp2 = T*[tmp;1];
        p(:,i) = tmp2(1:3);
    end
    
    for i=1:25
        tmp = [cm.x(i);cm.y(i);cm.z(i)];
        tmp2 = T*[tmp;1];
        cm.x(i)=tmp2(1);
        cm.y(i)=tmp2(2);
        cm.z(i)=tmp2(3);
    end
    
    tmp = [com.x;com.y;com.z];
    tmp2 = T*[tmp;1];
    com.x=tmp2(1);
    com.y=tmp2(2);
    com.z=tmp2(3);
    
    X = p(1,1:17);
    Y = p(2,1:17);
    Z = p(3,1:17);
    
    X1 = [p(1,12), p(1,18:22)];
    Y1 = [p(2,12), p(2,18:22)];
    Z1 = [p(3,12), p(3,18:22)];
    %
    X2 = [p(1,8),p(1,23:29)];
    Y2 = [p(2,8),p(2,23:29)];
    Z2 = [p(3,8),p(3,23:29)];
    
    [xs,ys,zs] = sphere;
    
    
    if(count == 0)
        figure(2)
        plot3(X,Y,Z,'k.-');
        hold on;
        plot3(X,Y,Z,'bo');
        plot3(X1,Y1,Z1,'k.-');
        plot3(X1,Y1,Z1,'bo');
        plot3(X2,Y2,Z2,'k.-');
        plot3(X2,Y2,Z2,'bo');
        plot3(cm.x(1:25),cm.y(1:25),cm.z(1:25),'co');
        plot3(com.x,com.y,com.z,'ro');
        axis equal
        grid on
        pause(0.1)
        
    else
        figure(2)
        plot3(X,Y,Z,'r.-');
        hold on;
        plot3(X,Y,Z,'ko');
        plot3(X1,Y1,Z1,'r.-');
        plot3(X1,Y1,Z1,'ko');
        plot3(X2,Y2,Z2,'r.-');
        plot3(X2,Y2,Z2,'ko');
        plot3(cm.x(1:15),cm.y(1:15),cm.z(1:15),'co');
        plot3(com.x,com.y,com.z,'ro');
        axis equal
        grid on
        pause(0.1)
        
    end
    
end

end