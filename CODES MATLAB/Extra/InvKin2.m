function [q,Dq,pos] = InvKin2(x,y,z)
syms theta1 theta2 theta3 theta4

theta = [180, -90, -90, 90]; %Default thetas = [180, -90, -90, 90]

T_0_1 = Trans_syms(theta1,0,20,-90);
T_1_2 = Trans_syms(theta2,0,-33.7,-90);
T_2_3 = Trans_syms(theta3,10,-148.4,-90);
T_3_4 = Trans_syms(theta4,120,0,0);

% T_0_1 = Hom_Trans(-90,0,20,theta(1));
% T_1_2 = Hom_Trans(-90,0,-33.7,theta(2));
% T_2_3 = Hom_Trans(-90,10,-148.4,theta(3));
% T_3_4 = Hom_Trans(0,120,0,theta(4));

T_final = T_0_1*T_1_2*T_2_3*T_3_4;

%Jacobian calculation

f1 = T_final(1,4);
f2 = T_final(2,4);
f3 = T_final(3,4);

J(theta1,theta2,theta3,theta4) = jacobian([f1 f2 f3],[theta1 theta2 theta3 theta4]);

%Inverse Kinematics Calculation

[u,p,T_final,pts] = ForwKin(theta);
I = eye(4);
theta_0 = theta; 

%inverse Kinematics
for ang=1:length(x)
    t(:,ang) = [x(ang);y(ang);z(ang)];
    disp(ang)
    sdq = [0 0 0 0];
    for i=1:100
        pre = p(:,5);
        err = t(:,ang)-pre;
        if ( abs(err(1)) < 0.001 && abs(err(2)) < 0.001 && abs(err(3)) < 0.001)
            q(:,ang) = theta;
            Dq(:,ang) = rad2deg(sdq);
            pos(:,ang) = pre;
            break;
        else
            J1 = double(J(deg2rad(theta(1)),deg2rad(theta(2)),deg2rad(theta(3)),deg2rad(theta(4))));
            J2 = (J1'*(J1*J1')^-1);
            %dq = J2*err + (I-J2*J1)*(theta-theta_0)';
            dq = J2*err;
            sdq = sdq + (double(dq'));
            theta= theta+rad2deg(double(dq'));
            [u,p,T_final,pts] = ForwKin(theta);
        end
    end
end
end

