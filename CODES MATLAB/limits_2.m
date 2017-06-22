%#####################################################################################################
%# Function : Angle limits for each base                                                             #   
%#                                                                                                   #       
%# Input(s)  : str ('hips','rleg','lleg' )                                                           #
%#                                                                                                   #                
%# Ouptut(s) : Angle limits                                                                          # 
%#                                                                                                   #       
%# Example: limits_2('rleg')                                                                #   
%#                                                                                                   #       
%#                                                                                                   #               
%#                                                                                                   #                                       
%#####################################################################################################




function [mi] = limits_2(str)
%hip limits
ang_min = [1300,1800,1200,1250,2200,10,3200,1000,650,2200,10,3000,3000,3300,1300,2300,650,3300,2050,1800,1350,1300,1800,3300,3300,2050,1800,2850]; %change this for original robot
ang_max = [2800,2650,2850,2850,1700,4050,1050,3000,2030,1700,4050,850,1000,2030,2800,1800,3300,1800,1450,3000,2850,2800,2300,650,1800,1450,3000,1350];%change this for original robot

if(str == 'hips')
    %     t_min = [1300,1800,1200,1250,2200,10,3200,1000,650,2200,10,3000,3000,3300,1300,2300,650,3300,2050,1800,1350,1300,1800,3300,3300,2050,1800,2850]; %change this for original robot
    %     t_max = [2800,2650,2850,2850,1700,4050,1050,3000,2030,1700,4050,850,1000,2030,2800,1800,3300,1800,1450,3000,2850,2800,2300,650,1800,1450,3000,1350];%change this for original robot
    
    t_min = ang_min;
    t_max = ang_max;
    mi(1,:) = (t_min - 2048).*(360/4096);
    mi(2,:) = (t_max - 2048).*(360/4096);
    
    for i=1:28
        if(mi(1,i)>mi(2,i))
            mi(1,i) = -mi(1,i);
            mi(2,i) = -mi(2,i);
        end
    end
    mi(2,14) = -mi(2,14);
    
elseif (str=='rleg')
    %write this
    
    t_min = zeros(1,28);
    t_max = zeros(1,28);
    
    
    
    t_min(7:15) = ang_min(1:9);
    t_min(16:20) = ang_min(10:14);
    t_min(21:27) = ang_min(15:21);
    t_min(1:6) =  fliplr(ang_min(23:28) )  ;
    
    
    t_max(7:15) = ang_max(1:9);
    t_max(16:20) = ang_max(10:14);
    t_max(21:27) = ang_max(15:21);
    t_max(1:6) =  fliplr( ang_max(23:28) )  ;
    
    
    
    mi(1,:) = (t_min - 2048).*(360/4096);
    mi(2,:) = (t_max - 2048).*(360/4096);
    
    for i=1:28
        if(mi(1,i)>mi(2,i))
            mi(1,i) = -mi(1,i);
            mi(2,i) = -mi(2,i);
        end
    end
    
    tmp = mi(1,1:6);
    mi(1,1:6) = -mi(2,1:6);
    mi(2,1:6) = -tmp;
    
    
    mi(2,20) = -mi(2,20);
    
    
    
    
else
    t_min = zeros(1,28);
    t_max = zeros(1,28);
    
    
    t_min(1:6) =  fliplr(ang_min(16:21) );
    t_min(7:11) = ang_min(1:5);
    t_min(12:15) = ang_min(11:14);
    t_min(16) = t_min(11);
    t_min(17:20) = ang_min(6:9);
    t_min(21) = t_min(7);
    t_min(22:27) = ang_min(23:28);
    
    t_max(1:6) =  fliplr(ang_max(16:21) );
    t_max(7:11) = ang_max(1:5);
    t_max(12:15) = ang_max(11:14);
    t_max(16) = t_max(11);
    t_max(17:20) = ang_max(6:9);
    t_max(21) = t_max(7);
    t_max(22:27) = ang_max(23:28);
    
    
    
    mi(1,:) = (t_min - 2048).*(360/4096);
    mi(2,:) = (t_max - 2048).*(360/4096);
    
    for i=1:28
        if(mi(1,i)>mi(2,i))
            mi(1,i) = -mi(1,i);
            mi(2,i) = -mi(2,i);
        end
    end
    
    tmp = mi(1,1:6);
    mi(1,1:6) = -mi(2,1:6);
    mi(2,1:6) = -tmp;
    
    mi(2,15) = -mi(2,15);
    
    
end

end

