%#####################################################################################################
%# Function : To generate the trajectectory                                                          #   
%#                                                                                                   #       
%# Input(s)  : None                                                                                  #
%#                                                                                                   #                
%# Ouptut(s) : Points on the trajectory                                                              # 
%#                                                                                                   #       
%# Example: Traj_4()                                                                                 #   
%#                                                                                                   #       
%#                                                                                                   #               
%#                                                                                                   #                                       
%#####################################################################################################

function [x,y,z] = Traj_4()

%Trajectory3
c = [20,150,0];
r = 30;
x = [];
y = [];
z = [];
for i=0:5:360
    z = [z,c(3)+r*sind(i)];
    y = [y,c(2)+r*cosd(i)];
    x = [x,-abs(sqrt(c(1)^2-r^2))];
end

end
