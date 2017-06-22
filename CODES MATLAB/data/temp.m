
% for i = 1:length(axis_loc)
% axis(i,:) = str2num(cell2mat(axis_loc(i)));
% end
% for i= 1:length(com)
% CoM(i,:) = str2num(cell2mat(com(i)));
% end

clear x y z;

last = 4;
len  = 3;
for i=1:last
    x(i) = CoM(i+6+4,1)-axis(len,1);
    y(i) = CoM(i+6+4,2)-axis(len,2);
    z(i) = CoM(i+6+4,3)-axis(len,3);
end

var = 11;
load com_data
cm_data(var).axis = axis(len,:);
cm_data(var).x = x;
cm_data(var).y = y;
cm_data(var).z = z;
cm_data(var).mass = Mass(11:14);