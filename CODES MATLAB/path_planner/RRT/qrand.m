function [ q_rand] = qrand(g)
rn = 4000;
x = rand(1);
if(x<0.95)
    sn = ((-2)*rand + 1);
    q_rand(1)=(sn/abs(sn))*((rn-1).*rand(1,1) + 1);
    sn = ((-2)*rand + 1);
    q_rand(2)=(sn/abs(sn))*((rn-1).*rand(1,1) + 1);
    sn = ((-2)*rand + 1);
    q_rand(3)=(sn/abs(sn))*((rn-1).*rand(1,1) + 1);
else
    q_rand = g;
end

end

