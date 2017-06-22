clc;
clear all;
close all;

l1 = 100;
l2 = 80;
l3 = 90;

m1 = 100;
m2 = 90;
m3 = 120;

cm_x = [l1/2,l2/2,l3/2];
cm_y = [0,0,0];
cm_z = [0,0,0];

th1 = 10;
th2 = 20;
th3 = 20;

[x,y,z,cm_x,cm_y,cm_z] =  Forw_link2(l1,l2,l3,cm_x,cm_y,cm_z,th1,th2,th3);

comx = (m1*cm_x(1)+m2*cm_x(3)+m3*cm_x(3))/(m1+m2+m3);
comy = (m1*cm_y(1)+m2*cm_y(3)+m3*cm_y(3))/(m1+m2+m3);
comz = (m1*cm_z(1)+m2*cm_z(3)+m3*cm_z(3))/(m1+m2+m3);

plot(x,y,'.-');
hold on;
plot(x,y,'ko');
plot(cm_x,cm_y,'go');
plot(comx,comy,'ro');
hold off;