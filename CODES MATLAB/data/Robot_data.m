% clc;
% clear all;
% close all;

%link lengths
l0 = 58.1;
l1 = 36;
l2 = 130.15;
l3 = 160.2;
l4 = 49.0;
l5 = 67.0;
l6 = 50.7;
l8 = 57.8;
l9 = 77.9;
l11 = 51.1;
l12 = 65.3;
l13 = 101.1;
l15 = 38.8;
l16 = 109.5;
l17 = 120;

%Offsets
d3 = 17.3;
d4 = 14;
d5 = 22.6;
d6 = 10.7;
d8 = 4.2;
d9 = 4.6;
d14 = 28.2;
d15 = 10;


%For Leg Joint
%alphas

alp1 = [0, 90, 0, 0, -90, 90, 0, 90, 90, 90, 90, 0, 90 ,-90, -90, 0]; %0 to 17
alp2 = [0, 90, -90, -90, 0]; % 13' to 17'
alp3 = [0, -90, -90, 0, 0 ,90, 0]; %5' - 00'

%thetas

th1 = [0, 0, -3.5682, 3.5682, -90, -90, 65.9746, 24.0254, 90, 90, 90, 0, 180, 90, -90, -90]; %0 to 17
th2 = [0, 180, 90, -90, -90]; %13' to 17'
th3 = [-131.9492, 65.9746, -90, 90, -3.5682, 3.5682, 0]; %5' - 00'

%For Hip Joint

%thetas

theta1 = [0, th1(9:16)];
theta2 = th2;
theta3 = [155.9746, 114.0254, -90, 90, -3.5682, 3.5682, 0]; 
theta4 = [-155.9746, 65.9746, -90, 90, -3.5682, 3.5682, 0];

%alphas

alpha1 = [90, 90, 90, 90, 0, 90 ,-90, -90, 0];
alpha2 = alp2;
alpha3 = alp3;
alpha4 = alp3;



