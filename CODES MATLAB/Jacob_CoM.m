%#####################################################################################################
%# Function : Jacobian for calculating IK of COM trajectory                                          #   
%#                                                                                                   #       
%# Input(s)  : None #
%#                                                                                                   #                
%# Ouptut(s) : Matrix (3 x 28)                                                # 
%#                                                                                                   #       
%# Example: Jacob_CoM()                                                              #   
%#                                                                                                   #       
%#                                                                                                   #               
%#                                                                                                   #                                       
%#####################################################################################################


function [ J_com ] = Jacob_CoM ()
global robot;
global base;

J_com = zeros(3,28);
M = 0;

if(base == 'hips')
for i=1:28
    
    if(i~=10 && i~=15 && i~=22)
        Idx = find_route(i,1);
        u = [robot.parts(Idx).Z_w];
        p = [robot.parts(Idx).axis_loc];
        
        l = length(Idx);
        r = robot.parts(i).com_g;
        m = robot.parts(i).mass;
        
        J{i}(1:3,28) = [0;0;0];
        %J{i}(4:6,28) = [0;0;0];
        for j = 1:l
            J{i}(1:3,Idx(j)) = cross(u(:,j), (r' - p(:,j)));
            %J{i}(4:6,Idx(j)) = u(:,j);
        end
       J_com = J_com + m*J{i};
       M = M + m;
   end   
end
J_com(:,10) = J_com(:,6);
J_com(:,15) = J_com(:,1);
J_com(:,22) = J_com(:,1);
J_com = J_com/M;

else
    for i=1:28
        
        if(i~=16 && i~=21 && i~=28)
            Idx = find_route(i,1);
            u = [robot.parts(Idx).Z_w];
            p = [robot.parts(Idx).axis_loc];
            
            l = length(Idx);
            r = robot.parts(i).com_g;
            m = robot.parts(i).mass;
            
            J{i}(1:3,28) = [0;0;0];
            %J{i}(4:6,28) = [0;0;0];
            for j = 1:l
                J{i}(1:3,Idx(j)) = cross(u(:,j), (r' - p(:,j)));
                %J{i}(4:6,Idx(j)) = u(:,j);
            end
            J_com = J_com + m*J{i};
            M = M + m;
        end
    end
    J_com(:,16) = J_com(:,11);
    J_com(:,21) = J_com(:,7);
    J_com = J_com/M;
    
end

end

