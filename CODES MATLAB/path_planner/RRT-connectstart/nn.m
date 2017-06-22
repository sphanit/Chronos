function [np] = nn(q_rand,path)

d=inf;

l=length(path.x);

for i=1:l
    p=sqrt((path.x(i)-q_rand(1))^2+(path.y(i)-q_rand(2))^2+(path.z(i)-q_rand(3))^2);
    if(p<d)
        d=p;
        np=i;
    end

end

