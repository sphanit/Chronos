clc;
clear all;
close all;

%ankle right (Number Convention is according to right leg base)

%Axis - 0
var = 1;
cm_data(var).axis = [16.85,120.53,163.07];
cm_data(var).mass = [140.2500   23.8700   17.5300   62.4500   10.1800    1.4500    1.2000];
com = [
        15.44, 78.37, 177.02
        22.21,74.03, 276.04
        23.18,64.55,278.76
        15.75,65.49,181.08
        16.83, 92.01, 145.66
        16.80,120.53,99.78
        16.85,120.53,191.28
    ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [7.96e+005, -1.6e+003, 3.27e+004;
                 -1.6e+003, 8.67e+005, -2.89e+004;
                  3.27e+004, -2.89e+004, 1.24e+005];
           
%Axis - 1
var = 2;
cm_data(var).axis = [16.85,156.53,163.07];
cm_data(var).mass = [70.6700   70.6700    0.5600    2.7100    0.4900    0.4900    0.0900    0.0900    0.2500    0.2500];
com = [
        16.82,136.11,122.25
        16.75,140.96,163.16
        16.85, 120.53, 187.88
        16.85,121.67,184.56
        34.76,127.53,144.25
        (-1.11), 127.53, 144.27
        16.85,120.53,192.32
        (-6.26), 156.53, 163.17
        16.85,120.53,190.85
        (-4.57),156.53,163.17
    ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [1.22e+005 837 507;
                  837 1e+005 5.38e+003;
                  507 5.38e+003 5.78e+004];
              
%Leg Right         
%Axis - 2
var = 3;
cm_data(var).axis = [16.85,286.45,155.01];
cm_data(var).mass = [80.7700    1.2000    1.6100    0.8400    1.6100    1.2000];
com = [
        17.80,232.64, 160.90
        40.85,286.45,155.08
        (-5.16) , 286.45,155.11
        (-2.91), 286.45,155.11
        40.59,156.53,163.15
        (-5.08), 156.53,163.17
    ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [1.98e+005 -2.17e+003 170;
                  -2.17e+003 7.84e+004 7.55e+003;
                  170 7.55e+003 2.24e+005];
              
              
%Axis - 3
var = 4;
cm_data(var).axis = [(-0.59),446.61,154.12];
cm_data(var).mass = [70.6700  120.2700    1.2000    1.6100];
com = [
        18.68,302.02,155.09
        11.82,371.46, 157.50
        23.41,446.61,154.21
        -22.40,446.61,154.23
    ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [600909.28, -67269.94, 589.20;
	              -67269.94, 140088.30, -9676.78;
                  589.20, -9676.78, 627740.86];
              

%Pelvis Changed
%Axis - 4
var = 5;
cm_data(var).axis = [(-0.59),495.61,140.22];
cm_data(var).mass = [70.6700    0.2500    0.0900    0.8400    1.6100    2.7100];
com = [
        1.43,446.61,138.64
        22.98,446.61,154.21
        24.45,446.61,154.21
        -20.15,446.61,154.23
        1.35,471.85,140.2
        1.35,468, 141.36
    ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [27196.52, 5.89, -446.80;
                  5.89, 30301.85, 175.64;
                  -446.80, 175.64, 19384.49];

              
%Axis - 5
var = 6;
cm_data(var).axis = [68.35,495.61,140.22];
cm_data(var).mass = [70.6700    1.2000    0.2500    0.0900    0.8400    1.2000    1.6100    2.7100   10.9100];
com = [
        1.34,495.69,124.64
        1.35,517.67,140.22
        1.35,517.24,140.22
        1.35,518.70,140.22
        1.35,474.11,140.22
        68.35,495.61,155.99
        68.33,495.61,110.18
        22.74,495.61,132.07
        38.32,495.60,133.20
    ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [39390.99, -162.66, 3674.20;
                  -162.66, 58635.88, -80.27;
                  3674.20, -80.27, 45543.19];
 
              
%Axis - 6 (has a fixed mass attached)
var = 7;
cm_data(var).axis = [91.72, 546.28, 150.97];
cm_data(var).mass = [151.9];
com = [
91.70,560.84,131.87
    ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [128630.79, 357.30, 68.32
               	  357.30, 107746.82, -2611.59;
                  68.32, -2611.59, 58262.63];
              
cm_data(var).mass_fixed = [73.7600   19.6400   70.6700    0.2500    0.0900    0.8400   70.6700    0.2500    0.0900    0.8400];
com = [
        90.94,514.45,136.32
        90.94,448.38,133.92
        113.54,480.03,134
        113.55,495.61,155.54
        113.55,495.61,157.01
        113.53,495.61,112.41
        68.34,480.03,134.02
        68.35,495.61,155.56
        68.35,495.61,157.03
        68.35,495.61,112.43
    ];
for i = 1:length(cm_data(var).mass_fixed)
    cm_data(var).x_fixed(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y_fixed(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z_fixed(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I_fixed = [193881.22, 9.18, -67.56;
                        9.18, 210732.23, 9010.94;
                        -67.56, 9010.94, 322993.09];
                    

%5DOF chest                    
%Axis - 7
var = 8;
cm_data(var).axis = [91.72, 546.28, 150.97];
cm_data(var).mass = [95.39 1.61];
com = [
        91.69,590.92,132.61
        91.70,604.43,146.75
    ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [148140.87, -0.10, 36.95;
                  -0.10, 184881.93, -226.54;
                   36.95, -226.54, 77204.7];
               
               
%Axis - 8
var = 9;
cm_data(var).axis = [91.72, 604.08, 146.75];
cm_data(var).mass = [36.59 70.67 0.84];
com = [
        91.69,646.84,131.06
        91.69,628.27,131.18
        91.70,606.68,146.75
    ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [72808.95, 1.66, 4.38;
                  1.66, 54590.87, 2586.17;
                  4.38, 2586.17, 65685.23];
 
%Axis - 9
var = 10;
cm_data(var).axis = [91.72, 681.98, 142.14];
cm_data(var).mass = [151.9];
com = [
        92.45,696.54,123.04
    ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [128630.79, 357.30, 68.32;
                  357.30, 107746.82, -2611.59;
                  68.32, -2611.59, 58262.63];

      
              
%Axis - 10
var = 11;
cm_data(var).axis = [91.72, 681.98, 142.14];
cm_data(var).mass = [247.69 73.12 73.12 42.79];
com = [
        92.44,723.70,121.49
        17.45,733.08,127.36
        167.42,733.08,127.28
        92.47,756.49,124.62
    ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [476705.61, 21.75, -678.40;
                  21.75, 1894715.21, -2326.70;
                  -678.40, -2326.70, 1737244.76];
              
%Arm_right              
%Axis - 13
var = 12;
cm_data(var).axis = [(-9.38), 733.08, 142.14];
cm_data(var).mass = [3.31];
com = [
        -15.47,733.08,142.43
      ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [1206.11, 0.00, 0.39;
                  0.00, 1303.32, 0.00;
                  0.39,  0.00, 527.74];
              
%Axis - 14
var = 13;
cm_data(var).axis = [(-37.58), 733.08, 142.14];
cm_data(var).mass = [45.19 7.77 0.79];
com = [
        (-36.85), 722.06,142.41
        (-36.86), 698.51,139.48
        (-36.86), 690.74,132.14
      ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [19274.16, 0.47, 1.48;
                  0.47,  11013.26, 879.11;
                  1.48, 879.11, 16469.74];
              
              
%Axis - 15
var = 14;
cm_data(var).axis = [(-37.58), 733.08, 142.14];
cm_data(var).mass = [43.26 0.28 45.19 18.18];
com = [
        (-36.85), 670.85,143.66
        -36.86,689.06,132.14
        (-37.13), 595.81,142.14
        (-36.85), 640.96,143.89
      ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [173493.04, 705.40, 13.10;
                  705.40, 29719.22, 3089.58;
                  13.10, 3089.58, 169587.95];  
              
              
%Axis - 16
var = 15;
cm_data(var).axis = [(-37.58), 584.78, 152.14];
cm_data(var).mass = [28.06];
com = [
        (-40.85), 537.33,131.47
      ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [50450.36, 4669.99, -608.99;
                  4669.99, 9962.36, -4996.80;
                  -608.99, -4996.80, 52570.13];
              
              
%Arm_left              
%Axis - 13'
var = 16;
cm_data(var).axis = [192.82, 733.08, 142.14];
cm_data(var).mass = [3.31];
com = [
        200.37,733.08,142.31
      ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [1206.11, 0.00, 0.32;
                  0.00, 1303.32, 0.00;
                  0.32, 0.00, 527.74];        
              
              
%Axis - 14'
var = 17;
cm_data(var).axis = [221.02, 733.08, 142.14];
cm_data(var).mass = [45.19 7.77 0.79];
com = [
        221.75,722.06,142.28
        221.74,698.51,139.35
        221.74,690.74,132.01
      ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [19274.16, 0.47, 1.48;
                  0.47, 11013.26, 879.11;
                  1.48, 879.11, 16469.74];
              
              
%Axis - 15'
var = 18;
cm_data(var).axis = [221.02, 733.08, 142.14];
cm_data(var).mass = [43.26 0.28 45.19 18.18];
com = [
        221.75,670.85,143.52
        221.74,689.06,132.01
        221.47,595.81,142.01
        221.75,640.96,143.76
      ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [173493.04, 705.40, 13.10;
                  705.40, 29719.22, 3089.58;
                  13.10, 3089.58, 169587.95];
              
              
%Axis - 16'
var = 19;
cm_data(var).axis = [221.02, 584.78, 152.14];
cm_data(var).mass = [28.06];
com = [
        225.12,537.33,131.33
      ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [50454.04, -4681.24, 603.57;
                  -4681.24, 9964.23, -4993.12;
                  603.57, -4993.12, 52577.06];    
              
              
%Left Part - Pelvic Changed
%Axis - 5'
var = 20;
cm_data(var).axis = [113.55,495.61,140.22];
cm_data(var).mass = [70.6700    1.2000    0.2500    0.0900    0.8400    1.2000    1.6100    2.7100   10.9100];
com = [
        180.54,495.69,124.55
        180.55,517.67,140.13
        180.55,517.24,140.13
        180.55,518.70,140.13
        180.55,474.11,140.13
        113.55,495.61,155.97
        113.53,495.61,110.15
        159.14,495.61,132
        143.57,495.62,133.15
    ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [39398.56, 158.08, -3680.61;
                  158.08, 58635.88, -78.78;
                  -3680.61, -78.78, 45535.39];


%Axis - 4'
var = 21;
cm_data(var).axis = [180.77,495.61,140.22];
cm_data(var).mass = [70.6700    0.2500    0.0900    0.8400    1.6100    2.7100];

com = [
        180.46,446.61,138.55
        158.92,446.61,154.14
        157.46,446.61,154.14
        202.05,446.61,154.11
        180.55,471.85,140.13
        180.55,468,141.27
    ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [27195.58, -5.70, 454.97;
                -5.70, 30301.85, 175.65;
                 454.97, 175.65, 19385.43]; 
             
             
%Axis - 3'
var = 22;
cm_data(var).axis = [180.77,446.61,154.12];
cm_data(var).mass = [70.67 120.27 1.2 1.61];
com = [
        163.22, 302.02,155.02
        170.17,370.80,157.51
        158.49,446.61,154.14
        204.31,446.61,154.11
    ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [590687.12, 68047.88, -703.99;
                  68047.88, 139804.59, -8879.66;
                  -703.99, -8879.66, 617657.29];            
 
%Axis - 2'
var = 23;
cm_data(var).axis = [163.33,286.45,155.01];
cm_data(var).mass = [80.7700    1.2000    1.6100    0.8400    1.6100    1.2000];
com = [
        164.05,232.62,160.82
        141.25,286.45,155.03
        187.06,286.45,155.01
        184.81,286.45,155.01
        187.07,156.53,163.07
        141.39,156.53,163.10
    ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [1.98e+005, 787, -151;
                  787, 7.83e+004, 7.53e+003;
                  -151, 7.53e+003, 2.24e+005];

              
%Axis - 1'
var = 24;
cm_data(var).axis = [163.33,156.53,163.07];
cm_data(var).mass = [70.67 70.67 0.56 2.71 0.49 0.49 0.09 0.09 0.25 0.25];
com = [
        163.23,140.96,163.09
        163.29,136.11,122.17
        163.33,120.53,187.80
        163.32,121.67,184.48
        145.37,127.53,144.20
        181.24,127.53,144.18
        163.33,120.53,192.24
        140.22,156.53,163.10
        141.82,156.53,163.10
        163.33,120.53,190.77
    ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I = [1.22e+005 837 507;
                  837 1e+005 5.38e+003;
                  507 5.38e+003 5.78e+004];
              
              
%Axis - 0'
var = 25;
cm_data(var).axis = [163.33,120.53,163.07];
cm_data(var).mass = [140.25 23.87 17.53 62.45 10.18 1.45 1.2];
com = [
        164.72,78.37,176.95
        158.05,74.03,275.97
        156.07,65.55,278.70
        164.42,65.49,181.00
        163.30,92.01,145.59
        163.28,120.53,99.71
        163.33,120.53,191.21
    ];
for i = 1:length(cm_data(var).mass)
    cm_data(var).x(i) = com(i,1) - cm_data(var).axis(1);
    cm_data(var).y(i) = com(i,2) - cm_data(var).axis(2);
    cm_data(var).z(i) = com(i,3) - cm_data(var).axis(3);
end
cm_data(var).I =  [7.96e+005 1.63e+003 -3.36e+004;
                   1.63e+003  8.68e+005 -2.74e+004;
                   -3.36e+004 -2.74e+004 1.24e+005];
              