function [nps] = near(Ts,q_new,delta)

k = 10*2*(1+1/3)^(1/3)*(3/(4*pi))^(1/3);

l=length(Ts.x);
if(l>1)
    d=min(k*(log(l)/l)^(1/3),delta);
else
    d = delta;
end
nps = [];

for i=1:l-1
    p=sqrt((Ts.x(i)-q_new(1))^2+(Ts.y(i)-q_new(2))^2+(Ts.z(i)-q_new(3))^2);
    if(p<=d)
        nps=[nps,i];
    end
    
end

