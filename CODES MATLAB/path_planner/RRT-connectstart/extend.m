function [Ts,status,theta,q_new,last] = extend(Ts,q_rand,delta,to,from,adj)

np = nn(q_rand,Ts);
q_nearest = [Ts.x(np),Ts.y(np),Ts.z(np)];
q_new = move(q_nearest,q_rand,delta);

u = (q_rand-q_nearest)/norm((q_rand-q_nearest));
c = [334.2977   57.0000   52.3999];
r=0;
theta = zeros(1,28);
% [flag,theta] = InvKin(q_new(1),q_new(2),q_new(3),tg,Tr_mat,to,from,theta);

% InvKin_com(x,y,z,tg,Tr_mat,enf,from,theta)

flag = 1;
last = 0;

if((dot(u,(q_nearest-c)))^2-(norm(q_nearest-c))^2+r^2 < 0 && flag)
    l=length(Ts.x)+1;
    
    Ts.x(l) = q_new(1);
    Ts.y(l) = q_new(2);
    Ts.z(l) = q_new(3);
    
    np_min = np;
    nps = near(Ts,q_new,delta);
    c_min = Ts.cost(np)+1*norm(q_nearest - q_new);
    
    l_np = length(nps);
    
    for i=1:l_np
        if(nps(i)~=np)
            if(Ts.cost(nps(i)) + 1*norm(q_new - [Ts.x(nps(i)),Ts.y(nps(i)),Ts.z(nps(i))]) < c_min)
                np_min = nps(i);
                c_min = Ts.cost(nps(i)) + 1*norm(q_new - [Ts.x(nps(i)),Ts.y(nps(i)),Ts.z(nps(i))]);
            end
        end
    end
    Ts.parent(l) = np_min;
    Ts.cost(l) = c_min;
    
    for i=1:l_np
        if(nps(i)~=np && (Ts.cost(l) + 1*norm(q_new - [Ts.x(nps(i)),Ts.y(nps(i)),Ts.z(nps(i))]) < Ts.cost(nps(i))))
            Ts.parent(nps(i)) = l;
            Ts.cost(nps(i)) = Ts.cost(l) + 1*norm(q_new - [Ts.x(nps(i)),Ts.y(nps(i)),Ts.z(nps(i))]);
        end
    end
    
    if(norm(q_new-q_rand) == 0)
        status = 1;
        last = 1;   
        return;
    else
        status = 2;
        return;
    end
end
status = 0;
return;
end

