function [q_new] = move(q_near,q_rand,delta)

if(norm(q_near-q_rand)<=5)
    q_new = q_rand;
else
    
    u = (q_rand-q_near)/norm((q_rand-q_near));
    q_new(1) = q_near(1) + delta*u(1);
    q_new(2) = q_near(2) + delta*u(2);
    q_new(3) = q_near(3) + delta*u(3);
end

end

