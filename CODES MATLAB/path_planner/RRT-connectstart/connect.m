function [Tg,st,theta,last] = connect(Tg,q,delta,to,from,adj)
theta = [];
et = 1;
while(1)
    et = et +1;
    [Tg,status,th,tmp,last]=extend(Tg,q,delta,to,from,adj);
    [sl sb] = size(th);
    if(sl+sb~=2)
    theta = [th,theta];
    end
    if(status ~= 2)
        st = status;
        break;
    end
end

end

