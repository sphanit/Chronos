function [ret,q,Dq] = get_reward(s,theta)
adj = [13,18];
[flag,q,Dq,pos,cdl,hG] = InvKin(s(1),s(2),s(3),16,8,adj,theta);

l = length(cdl);

if(l>1 && flag == 1)
    pt1 = [cdl(1).x,cdl(1).y,cdl(1).z];
    pt2 = [cdl(2).x,cdl(2).y,cdl(2).z];
    
    ret = (10 - 10*norm(pt2-pt1) - 10*hG(1) - 10*hG(2) - 10*hG(3))/10;
else
    ret = 0;
end

q = q';
Dq = Dq';
end
