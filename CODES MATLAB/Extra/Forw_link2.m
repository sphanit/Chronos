function [x,y,z,cm_x,cm_y,cm_z] = Forw_link2(l1,l2,l3,cm_x,cm_y,cm_z,th1,th2,th3)
T1 = Hom_Trans(0,l1,0,th1);
T2 = Hom_Trans(0,l2,0,th2);
T3 = Hom_Trans(0,l3,0,th3);

T_m1 = T1*T2;
T_m2 = T_m1*T3;

tmp = T1*[0;0;0;1];
p1 = tmp(1:3);
Z_w(1,:) = T1(1:3,1:3)*[0;0;1];
tmp = rotVecAroundArbAxis([cm_x(1),cm_y(1),cm_z(1)],[Z_w(1,:)],th1);
cm(1,:) = [tmp';1];


tmp = T_m1*[0;0;0;1];
p2 = tmp(1:3);
Z_w(2,:) = T_m1(1:3,1:3)*[0;0;1];

tmp = rotVecAroundArbAxis([cm_x(2),cm_y(2),cm_z(2)],Z_w(2,:),th2);
cm(2,:) = T1*[tmp(1);tmp(2);tmp(3);1];



tmp = T_m2*[0;0;0;1];
p3 = tmp(1:3);
Z_w(3,:) = T_m2(1:3,1:3)*[0;0;1];

tmp = rotVecAroundArbAxis([cm_x(3),cm_y(3),cm_z(3)],Z_w(3,:),th3);
cm(3,:) = T_m1*[tmp(1);tmp(2);tmp(3);1];


x = [0,p1(1),p2(1),p3(1)];
y = [0,p1(2),p2(2),p3(2)];
z = [0,p1(3),p2(3),p3(3)];

cm_x = [cm(1,1),cm(2,1),cm(3,1)];
cm_y = [cm(1,2),cm(2,2),cm(3,2)];
cm_z = [cm(1,3),cm(2,3),cm(3,3)];


end

