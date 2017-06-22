function [Q,Pol] = QAC(s,g,n)

disp('Learning Started');
%% Initialisation
Q = [];
Pol = [];
X = zeros(14,1);
a = zeros(14,1);

alpha = 0.1;
beta = 0.1;
gamma = 0.5;

w_q = rand(14,1);
w_pi = rand(14,1);

figure(1);

for i = 1:n
    it = 1;
    theta = zeros(1,28);
    theta(13) = -5;
    theta(18) = 5;
    
    theta(3) = -10;
    theta(2) = 10;
    
    theta(25)= 10;
    theta(26)= -10;
    
    %% Genrating an episode
    traj = rrt(s,g);
    np = length(traj);
    
    x1 = traj(1,:);
    y1 = traj(2,:);
    z1 = traj(3,:);
    
    hold on;
    plot3(x1,y1,z1,'b.-');
    pause(0.00001)
    hold off;
    
    fprintf('\n Running Iteration = %d\n',i);
    if(i==1)
        %% Initialising Q,Pol
        Q.s(:,1:np) = zeros(14,np);
        Q.a(:,1:np) = zeros(14,np);
        Q.val(1:np) = X'*w_q;
        
        Pol.s(:,1:np) = zeros(14,np);
        Pol.a(:,1:np) = zeros(14,np);
        Pol.val(1:np) = 0.1*exp(X'*w_pi);
        %% Updating
        X=deg2rad(theta(7:20)');
        Q.s(:,1) = X;
        Pol.s(:,1) = X;
        
        for j = 1:(np-1)
            fprintf(' .');
            [r,theta,del_theta] = get_reward(traj(:,j),theta);

            if(isempty(theta))
                break;
            end

            delta = r + gamma*Q.val(j+1) - Q.val(j);
            w_pi = w_pi + alpha*X*Q.val(j);
            w_q = w_q + beta*delta*X;
            
            w_pi = w_pi/norm(w_pi);
            w_q = w_q/norm(w_q);
            
            Q.val(j) = X'*w_q;
            Pol.val(j) = 0.1*exp(X'*w_pi);
            
            
            X = deg2rad(theta(7:20))';
            a = del_theta(7:20)';
            
            Q.s(:,j+1) = X;
            Q.a(:,j+1) = a;
            Pol.s(:,j+1) = X;
            Pol.a(:,j+1) = a;

        end
    else
        np_prev = length(Q.s)
        
        X=deg2rad(theta(7:20)');
        
        for j = 2:np
            [Y,id] = max(sum(bsxfun(@minus,rad2deg(Q.s(:,1:np_prev)),rad2deg(X))<0.1,1));
            
            if(Y == 14)
                
                fprintf(' .');
                
                Q_fut = 0;
                
                [r,theta,del_theta] = get_reward(traj(:,j),theta);
                
                if(isempty(theta))
                    break;
                end
                
                delta = r + gamma*Q_fut - Q.val(id);
                w_pi = w_pi + alpha*X*Q.val(id);
                w_q = w_q + beta*delta*X;
                
                w_pi = w_pi/norm(w_pi);
                w_q = w_q/norm(w_q);
                
                Q.val(id) = X'*w_q;
                Pol.val(id) = 0.1*exp(X'*w_pi);
                
                
                X = deg2rad(theta(7:20))';
                a = del_theta(7:20)';
                
            else
                
                % Updating States and Actions
                Q.s(:,np_prev+it) = X;
                Q.a(:,np_prev+it) = a;
                Q.val(np_prev+it) = X'*w_q;
            
                Pol.s(:,np_prev+it) = X;
                Pol.a(:,np_prev+it) = a;
                Pol.val(np_prev+it) = 0.1*exp(X'*w_pi);
                
                [r,theta,del_theta] = get_reward(traj(:,j),theta);

                if(isempty(theta))
                    break;
                end
                
                Q_fut = 0;
                
                delta = r + gamma*Q_fut - Q.val(np_prev+it);
                w_pi = w_pi + alpha*X*Q.val(np_prev+it);
                w_q = w_q + beta*delta*X;
                
                w_pi = w_pi/norm(w_pi);
                w_q = w_q/norm(w_q);
                
                Q.val(np_prev+it) = X'*w_q;
                Pol.val(np_prev+it) = 0.1*exp(X'*w_pi);
                
                X = deg2rad(theta(7:20))';
                a = del_theta(7:20)';
            end
                it = it +1;
        end
    end
    disp('.');
end
disp('Learning Ended');

end

