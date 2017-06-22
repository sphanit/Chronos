function [T] = Hom_Trans(alp,a,d,theta)

T = [cosd(theta), -(cosd(alp)*sind(theta)), sind(alp)*sind(theta), a*cosd(theta);
     sind(theta), cosd(alp)*cosd(theta), -(sind(alp)*cosd(theta)), a*sind(theta);
     0          , sind(alp)            , cosd(alp)             , d ;
     0          , 0                    , 0                     , 1            ];

end

