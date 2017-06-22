function [ q_rand] = qrand(g)
rn = 2000;
x = rand(1);
if(x<=0.9)
    q_rand(1)=((rn-1).*rand(1,1) + 1);
    q_rand(2)=((rn-1).*rand(1,1) + 1);
    q_rand(3)=((rn-1).*rand(1,1) + 1);
else
    q_rand = g;
end

end

