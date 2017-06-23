%#####################################################################################################
%# Function  : Generating leg and hip trajectories                                                   #   
%#                                                                                                   #       
%# Input(s)  : number of forward steps that has to be taken (n)                                      #
%#                                                                                                   #                
%# Ouptut(s) :x,z(leg trajectories),x_h,z_h(hip and COM trajectory)                                  # 
%#                                                                                                   #       
%# Example:new_traj(n)                                                                               #   
%#                                                                                                   #       
%#                                                                                                   #               
%#                                                                                                   #                                       
%#####################################################################################################


function [x,z,x_h,z_h] = new_traj(n)
x = [];
z = [];
x_h  = [];
z_h = [];
for k=0:n
    if(k>=1)
        Ds = 60;
        La = 0.7*Ds;
        Ha = 110;
    else
        Ds = 30;
        La = 0.9*Ds;
        Ha = 110;
    end
    lan = 94.1;
    lab = 60;
    laf = 90;
    qb = 10;
    qf = 10;
    
    Tc = 0.3;
    Td = 0.3*0.3;
    Tm = 0.5*0.3;
    
    
    
    xed = 0.4*Ds;
    xsd = 0.4*Ds;
    
    Hmin = -15;
    Hmax = -5;
    
    xa(1) = k*Ds;
    xa(2) = k*Ds + lan*sind(qb) + laf*(1-cosd(qb));
    xa(3) = k*Ds + La;
    xa(4) = (k+2)*Ds - lan*sind(qf)-lab*(1-cosd(qf));
    xa(5) = (k+2)*Ds;
    
    za(1) = lan;
    za(2) = laf*sind(qb) + lan*cosd(qb);
    za(3) = Ha;
    za(4) = lab*sind(qf)+lan*(cosd(qf));
    za(5) = lan;
    
    xh(1) = k*Ds + xed;
    xh(2) = ((k+1)*Ds) - xsd;
    xh(3) = ((k+1)*Ds) + xed;
    
    zh(1) = Hmin;
    zh(2) = Hmax;
    zh(3) = Hmin;
    
    t = [k*Tc ,k*Tc + Td, k*Tc + Tm, (k+1)*Tc, ((k+1)*Tc +Td) ];
    thx = [k*Tc, k*Tc + Td, (k+1)*Tc];
    thz = [(k*Tc + (Td/2)), (k*Tc + (Tc-Td/2)), (((k+1)*Tc) + (Td/2))];
    
    cs1 = spline(t,xa);
    cs2 = spline(t,za);
    tt = linspace(k*Tc,((k+1)*Tc+Td),100);
    
    cs3 = spline(thx,xh);
    cs4 = spline(thz,zh);
    tth2 = linspace(k*Tc + Td/2,(((k+1)*Tc) + Td/2),80);
    tth1 = linspace(k*Tc,(((k+1)*Tc)),80);
    figure(1)
    hold on;
    plot(ppval(cs1,tt),ppval(cs2,tt),'yo');
    
    z = [z,ppval(cs1,tt)];
    x = [x,ppval(cs2,tt)];
    
    
    zh_temp = ppval(cs3,tth1);
    xh_temp = ppval(cs4,tth2);
    
    if(k>=1)
        zh_temp = zh_temp-z_h(100);
    end
    zt1(1:20) = zh_temp(80);
    z_h = [z_h, zh_temp,zt1];
    
    xt1(1:10) = xh_temp(1);
    xt2(1:10) = xh_temp(80);
    x_h = [x_h,xt1, xh_temp,xt2];
    %hold off; plot(z_h,x_h,'r.');
    
end
end