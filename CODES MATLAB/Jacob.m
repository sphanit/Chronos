function [J] = Jacob(u,p)

l = length(p);
for i = 1:(l-1)
    J(1:3,i) = cross(u(:,i), (p(:,l)-p(:,i)));
    J(4:6,i) = u(:,i);
end

end

