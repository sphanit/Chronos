function [traj,theta_final,lol] = rrt_connect(start,goal,to,from,adj)
Ts.x(1) = start(1);
Ts.y(1) = start(2);
Ts.z(1) = start(3);
Ts.parent(1)=1;
Ts.cost(1)=0;

Tg.x(1) = goal(1);
Tg.y(1) = goal(2);
Tg.z(1) = goal(3);
Tg.parent(1)= 1;
Tg.cost(1) = 0;

delta = 2;
theta_final = [];
dist = 99999999;
for it = 1:1000
    gx=Tg.x(1);
    gy=Tg.y(1);
    gz=Tg.z(1);
    g = [gx,gy,gz];
    q_rand = qrand(g);
    [Ts,status,theta,q_new,last] = extend(Ts,q_rand,delta,to,from,adj);
    if(status ~= 0)
        [Tg,st,theta,last] = connect(Tg,q_new,delta,to,from,adj);
        [sl sb] = size(theta);
        if(sl+sb~=2)
            theta_final = [theta_final, theta];
        end
        if(st == 1)
            if(last == 1)
                tmp = Tg.cost(length(Tg.x))+Ts.cost(length(Ts.x));
                if(tmp<dist)
                    
                    Tg_f = [];
                    Ts_f = [];
                    dist = tmp;
                    
                    lgp = Tg.parent(length(Tg.x));
                    idxg = [lgp];
                    while(lgp~=1)
                    lgp = Tg.parent(lgp);
                    idxg = [lgp idxg];
                    end
                    
                    
                    lsp = Ts.parent(length(Ts.x));
                    idxs = [lsp];
                    while(lsp~=1)
                    lsp = Ts.parent(lsp);
                    idxs = [lsp idxs];
                    end
                    
                    Tg_f.x = Tg.x(idxg);
                    Ts_f.x = Ts.x(idxs);
                    
                    Tg_f.y = Tg.y(idxg);
                    Ts_f.y = Ts.y(idxs);
                    
                    Tg_f.z = Tg.z(idxg);
                    Ts_f.z = Ts.z(idxs);
                    
                    fprintf('dist = %.5f',dist);
                
                end
            end
        end
    end
    [sl sb] = size(theta);
    
    if(sl+sb~=2)
        theta_final = [theta_final, theta];
    end
    temp = Ts;
    Ts = Tg;
    Tg = temp;
    
end

tmp = [Ts_f.x(1),Ts_f.y(1),Ts_f.z(1)];

if(norm(tmp - goal) == 0)
    temp = Ts_f;
    Ts_f = Tg_f;
    Tg_f = temp;
end
%[flag,theta] = InvKin(goal(1),goal(2),goal(3),to,from,adj);
% [sl sb] = size(theta);
%     
%     if(sl+sb~=2)
% theta_final = [theta_final, theta];
%     end
theta = zeros(1,28);

l=length(Ts_f.x)-1;

traj = [Ts_f.x(:,2:l),fliplr(Tg_f.x);Ts_f.y(:,2:l),fliplr(Tg_f.y);Ts_f.z(:,2:l),fliplr(Tg_f.z)];

disp(traj);
end


