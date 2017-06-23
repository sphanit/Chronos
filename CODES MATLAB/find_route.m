  %#####################################################################################################
%# Function : Finds the route from "from"(input) to the part id                                      #   
%#                                                                                                   #       
%# Input(s)  : from (starting link of IK calculation), part_Id                                       #
%#                                                                                                   #                
%# Ouptut(s) : Idx(link indices present in the chain)                                                # 
%#                                                                                                   #       
%# Example: find_route(15,7) { eg: 15 is the end effector of the right arm, 7 is pelvis }             #   
%#                                                                                                   #       
%#                                                                                                   #               
%#                                                                                                   #                                       
%#####################################################################################################


function [Idx] = find_route(part_Id,from)
global robot
if(part_Id==from)
    Idx = [part_Id];
else
    Idx = [find_route(robot.parts(part_Id).P_Id,from) part_Id];
end

