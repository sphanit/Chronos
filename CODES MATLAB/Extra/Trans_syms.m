function [T] = Trans_syms(theta,a,d,alp)
T= [cos(theta), -cosd(alp)*sin(theta), sind(alp)*sin(theta), a*cos(theta);
     sin(theta), cosd(alp)*cos(theta), -sind(alp)*cos(theta), a*sin(theta);
     0          , sind(alp)            , cosd(alp)             , d ;
     0          , 0                    , 0                     , 1      
    ];

end

