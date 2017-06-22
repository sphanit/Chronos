%#####################################################################################################
%# Function : generate the stick figures                                                             #   
%#                                                                                                   #       
%# Input(s)  : p(position of the axis)                                                               #
%#                                                                                                   #                
%# Ouptut(s) : stick figure plot is obtained                                                         # 
%#                                                                                                   #       
%# Example: sample_draw(position)                                                                    #   
%#                                                                                                   #       
%#                                                                                                   #               
%#                                                                                                   #                                       
%#####################################################################################################




function [] = sample_draw(p)
%For Drawing in zero theta
global count
global base

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
        axis equal
        
    end
else
    %For Leg
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
        figure(1)
        plot3(X,Y,Z,'k.-');
        hold on;
        plot3(X,Y,Z,'bo');
        plot3(X1,Y1,Z1,'k.-');
        plot3(X1,Y1,Z1,'bo');
        plot3(X2,Y2,Z2,'k.-');
        plot3(X2,Y2,Z2,'bo');
        axis equal
        grid on
        pause(0.1)
        hold off;
    else
        figure(1)
        plot3(X,Y,Z,'r.-');
        hold on;
        plot3(X,Y,Z,'ko');
        plot3(X1,Y1,Z1,'r.-');
        plot3(X1,Y1,Z1,'ko');
        plot3(X2,Y2,Z2,'r.-');
        plot3(X2,Y2,Z2,'ko');
        axis equal
        grid on
        pause(0.1)
        hold off;
    end
    
end
end