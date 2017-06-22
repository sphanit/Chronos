function ji = invsvd(j)

[u,d,v] = svd(j);
% di = d'*inv(d*d');
di = zeros(size(d,2), size(d,1));

for i = 1:size(d,1)
    
    if d(i,i)~=0
        di(i,i) = 1/d(i,i);
    end
end


ji = v*di*(u');
end

