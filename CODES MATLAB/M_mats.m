function[h,AG,hG,vG] = M_mats(Tr_mat,com,dq)

global robot;

I_c = zeros(6,6,28);

for i = 1:28
    I_c(:,:,i) = robot.parts(i).Is;
end

for i = 28:-1:2
    p = robot.parts(i).P_Id;
%     fprintf('p=%d i=%d\n',p,i);
    I_c(:,:,p) = I_c(:,:,p) + robot.parts(i).Xs' * I_c(:,:,i) * robot.parts(i).Xs;
end

tmp = Tr_mat*[com.x;com.y;com.z;1];

X_0G = [eye(3)           zeros(3);
        hat(tmp(1:3))    eye(3)];

AG = [];
h = [];
hG = zeros(6,1);

for i = 1:28
    p = robot.parts(i).P_Id;
    if(p<1)
        Xpg = eye(6);
    else
        Xpg = robot.parts(p).Xpg;
    end
    
    X_iG = robot.parts(i).Xs * Xpg;
    AG_i = X_iG' * I_c(:,:,i) * [0;0;1;0;0;0];

    hG = hG + AG_i * dq(i);
    h = [h,AG_i * dq(i)];
    AG = [AG,AG_i];
    
end
    Ig = X_0G'*I_c(:,:,1)*X_0G;
    vG = ((Ig)^-1 )* hG;

end