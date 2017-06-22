%#####################################################################################################
%# Function : simulation function                                                                                                           #   
%#                                                                                                                                          #       
%# Input(s)  : x(x trajectory), y(y trajectory), z(z trajetcory),q( angular trajectory obtained from IK), loop(flag)                        #
%#                                                                                                                                          #                
%# Ouptut(s) : takes the inverse kinetics angles and runs the forward kinematcs on it so as to enable the robot reach the req position      #      # 
%#                                                                                                                                          #       
%# Example: follow_traj(traj1_matrix, traj2_matrix, traj3_matrix,angtraj_matrix,1)                                                          #      #   
%#                                                                                                                                          #       
%#                                                                                                                                          #               
%#                                                                                                                                          #                                       
%#####################################################################################################

function [] = follow_traj(x,y,z,q,loop)
global robot
global sim
global base
global steps
global move_base

r = 10;
T = [0 0 -1 0;0 1 0 0; 1 0 0 483.898;0 0 0 1];
T=eye(4);
centre = T*[414.2977 ; 102.0000 ;  12.3999 ;1];
dq = diff(q,1,2);
dq = [dq,zeros(28,1)];

[tp1,tp2,x_h,z_h] = new_traj(2*steps-1);

if(sim == 1)
    if (base == 'rleg')
        T = [0 0 -1 0;0 1 0 0; 1 0 0 58.1;0 0 0 1];
        %T= eye(4);
        for i=1:length(x)
            p(:,i) = T*[x(i);y(i);z(i);1];
        end
        x = p(1,:);
        y = p(2,:);
        z = p(3,:);
        while(1)
            count = 1;
            lq = size(q);
            lq = lq(2);
            for i=1:lq
                if(move_base == 1)
                    Tr_mat = [1 0 0 x_h(i);
                        0 1 0 0;
                        0 0 1 z_h(i);
                        0 0 0 1];
                else
                    Tr_mat =eye(4);
                end
                
                [com,c] = ForwKin(q(:,i)',Tr_mat,dq(:,i));
                cm = T*[com.x;com.y;com.z;1];
                for kk=1:25
                    ha = [c.x(kk);c.y(kk);c.z(kk);1];
                    ha = T*ha;
                    c.x(kk) = ha(1);
                    c.y(kk) = ha(2);
                    c.z(kk) = ha(3);
                end
                com.x = cm(1);
                com.y = cm(2);
                com.z = cm(3);
                
                p = [robot.draw];
%                 for kk = 1:28
%                     tmp = T*[p(1,kk);p(2,kk);p(3,kk);1];
%                     p(:,kk) = tmp(1:3);
%                 end
                
                X = p(1,1:17);
                Y = p(2,1:17);
                Z = p(3,1:17);
                
                X1 = [p(1,12), p(1,18:22)];
                Y1 = [p(2,12), p(2,18:22)];
                Z1 = [p(3,12), p(3,18:22)];
                
                X2 = [p(1,8),p(1,23:29)];
                Y2 = [p(2,8),p(2,23:29)];
                Z2 = [p(3,8),p(3,23:29)];
                
                [xs,ys,zs] = sphere;
                
                figure(1)
                plot3(X,Y,Z,'k.-');
                hold on;
                plot3(x,y,z);
                plot3(X,Y,Z,'bo');
                plot3(X1,Y1,Z1,'k.-');
                plot3(X1,Y1,Z1,'bo');
                plot3(X2,Y2,Z2,'k.-');
                plot3(X2,Y2,Z2,'bo');
                plot3(com.x,com.y,com.z,'ro');
                plot3(com.x,com.y,0,'go');
                plot3(c.x,c.y,c.z,'co');
                surf(xs*r+centre(1), ys*r+centre(2), zs*r+centre(3));
                hold off;
                axis([-300 300 -300 300 0 800])
                grid on
                axis equal;
                pause(0.1)
                F(count) = getframe;
                count=count+1;
            end
            if(loop == 0)
                lq = size(q);
                lq = lq(2);
                for i=lq:-1:1
                    Tr_mat = [1 0 0 x_h(i);
                        0 1 0 0;
                        0 0 1 z_h(i);
                        0 0 0 1];
                    [com,c] = ForwKin(q(:,i)',Tr_mat,dq(:,i));
                    for kk=1:25
                        ha = [c.x(kk);c.y(kk);c.z(kk);1];
                        ha = T*ha;
                        c.x(kk) = ha(1);
                        c.y(kk) = ha(2);
                        c.z(kk) = ha(3);
                    end
                    cm = T*[com.x;com.y;com.z;1];
                    com.x = cm(1);
                    com.y = cm(2);
                    com.z = cm(3);
                    
                    p = [robot.draw];
                    for kk = 1:28
                        tmp = T*[p(1,kk);p(2,kk);p(3,kk);1];
                        p(:,kk) = tmp(1:3);
                    end
                    
                    X = p(1,1:17);
                    Y = p(2,1:17);
                    Z = p(3,1:17);
                    
                    X1 = [p(1,12), p(1,18:22)];
                    Y1 = [p(2,12), p(2,18:22)];
                    Z1 = [p(3,12), p(3,18:22)];
                    
                    X2 = [p(1,8),p(1,23:29)];
                    Y2 = [p(2,8),p(2,23:29)];
                    Z2 = [p(3,8),p(3,23:29)];
                    
                    [xs,ys,zs] = sphere;
                    
                    figure(1)
                    plot3(X,Y,Z,'k.-');
                    hold on;
                    plot3(x,y,z);
                    plot3(X,Y,Z,'bo');
                    plot3(X1,Y1,Z1,'k.-');
                    plot3(X1,Y1,Z1,'bo');
                    plot3(X2,Y2,Z2,'k.-');
                    plot3(X2,Y2,Z2,'bo');
                    plot3(com.x,com.y,com.z,'ro');
                    plot3(com.x,com.y,0,'go');
                    plot3(c.x,c.y,c.z,'co');
                    surf(xs*r+centre(1), ys*r+centre(2), zs*r+centre(3));
                    hold off;
                    axis([-300 300 -300 300 0 800])
                    grid on
                    axis equal;
                    pause(0.001)
                    F(count) = getframe;
                    count=count+1;
                end
            end
            %  movie2avi(F,'new.avi');
        end
        
    else
        T = [0 0 -1 0;0 1 0 0; 1 0 0 483.898;0 0 0 1];
        %T = eye(4);
        for i=1:length(x)
            p(:,i) = T*[x(i);y(i);z(i);1];
        end
        x = p(1,:);
        y = p(2,:);
        z = p(3,:);
        while(1)
            count = 1;
            lq = size(q);
            lq = lq(2)
            for i=1:lq
                Tr_mat = [1 0 0 x_h(i);
                    0 1 0 0;
                    0 0 1 z_h(i);
                    0 0 0 1];
                [com,P,L,c] = ForwKin(q(:,i)',Tr_mat,dq(:,i));
                cm = T*[com.x;com.y;com.z;1];
                %                 p_cm = T*[P(1);P(2);P(3);1];
                %                 l_cm = T*[L(1);L(2);L(3);1];
                %
                %                 tmp1(:,i) = P;
                %                 tmp2(:,i) = L;
                %                 tmp3(:,i) = [com.x;com.y;com.z;1];
                
                for kk=1:25
                    ha = [c.x(kk);c.y(kk);c.z(kk);1];
                    ha = T*ha;
                    c.x(kk) = ha(1);
                    c.y(kk) = ha(2);
                    c.z(kk) = ha(3);
                end
                com.x = cm(1);
                com.y = cm(2);
                com.z = cm(3);
                
                p = [robot.draw];
                
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
                [xs,ys,zs] = sphere;
                
                figure(1)
                plot3(X,Y,Z,'k.-');
                hold on;
                plot3(x,y,z,'b.');
                plot3(X,Y,Z,'bo');
                plot3(X1,Y1,Z1,'k.-');
                plot3(X1,Y1,Z1,'bo');
                plot3(X2,Y2,Z2,'k.-');
                plot3(X2,Y2,Z2,'bo');
                plot3(X3,Y3,Z3,'k.-');
                plot3(X3,Y3,Z3,'bo');
                % surf(xs*r+centre(1), ys*r+centre(2), zs*r+centre(3));
                plot3(com.x,com.y,com.z,'ro');
                %plot3(com.x,com.y,-400,'go');
                plot3(c.x(1:25),c.y(1:25),c.z(1:25),'co');
                hold off;
                %axis([-300 300 -300 300 -450 350])
                grid on
                axis equal
                pause(0.0001)
                F(count) = getframe;
                count=count+1;
            end
            if(loop == 0)
                lq = size(q);
                lq = lq(2);
                
                for i=lq:-1:1
                    Tr_mat = [1 0 0 x_h(i);
                        0 1 0 0;
                        0 0 1 z_h(i);
                        0 0 0 1];
                    [com,c] = ForwKin(q(:,i)',Tr_mat,dq(:,i));
                    cm = Tr_mat*T*[com.x;com.y;com.z;1];
                    for kk=1:25
                        ha = [c.x(kk);c.y(kk);c.z(kk);1];
                        ha = T*ha;
                        c.x(kk) = ha(1);
                        c.y(kk) = ha(2);
                        c.z(kk) = ha(3);
                    end
                    com.x = cm(1);
                    com.y = cm(2);
                    com.z = cm(3);
                    
                    p = [robot.draw];
                    
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
                    [xs,ys,zs] = sphere;
                    
                    figure(1)
                    
                    plot3(X,Y,Z,'k.-');
                    hold on;
                    plot3(x,y,z,'b.');
                    plot3(X,Y,Z,'bo');
                    plot3(X1,Y1,Z1,'k.-');
                    plot3(X1,Y1,Z1,'bo');
                    plot3(X2,Y2,Z2,'k.-');
                    plot3(X2,Y2,Z2,'bo');
                    plot3(X3,Y3,Z3,'k.-');
                    plot3(X3,Y3,Z3,'bo');
                    % surf(xs*r+centre(1), ys*r+centre(2), zs*r+centre(3));
                    %plot3(com.x,com.y,com.z,'ro',p_cm(1),p_cm(2),p_cm(3),'mo',l_cm(1),l_cm(2),l_cm(3),'yo');
                    plot3(com.x,com.y,-400,'go');
                    %plot3(c.x,c.y,c.z,'co');
                    hold off;
                    axis([-300 300 -300 300 -450 350])
                    grid on
                    axis equal
                    pause(0.0001)
                    F(count) = getframe;
                    count=count+1;
                end
            end
            %  movie2avi(F,'walking.avi');
            break;
        end
        
    end
    
    
end
end

