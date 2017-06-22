% function Walking
%(Tsup, sx,sy) 

itr = 20;
itr=itr-1;
sx = [0.0 0.15 0.15 0.15 0];
sy =[0.15 0.15 0.15 0.15 0.15];

% sx = [0.0 0.15 0.15 0.15 0];
% sy = [0.15 0.2 0.1 0.2 0.15];

Tsup = 0.30;

Tc = sqrt(0.45/10);

T=0;
n=0;
px=0;
py=0;
global robot;
global base;
global time;


% 
% robot = make_model();
% theta=zeros(1,28);
% com = ForwKin(theta,eye(4),zeros(1,28));





%initialisation (change)
xi = com.z/1000;
yi = -com.y/1000;
plot(xi,yi,'square');
hold on;
% xi = -0.0157;
% yi = 0.07;
% x = sx(n+1)/2;
% y = ((-1)^n) * sy(n+1)/2;
xi_vel = 0.12;
yi_vel = -0.2;

px_mod = 0;
py_mod = 0;
 
px = px_mod;
py = py_mod;

COM_x=[];
COM_y=[];



for i=1:length(sx)
    
    
    
    
    plot(px_mod,py_mod, 'o');
    plot(px,py, '*');
    hold on;
    axis equal;
         
    x_traj = [];
    y_traj = [];
    for j=0:itr
        xtmp = (xi-px_mod)*cosh( (j/itr)*(Tsup/Tc) ) + Tc*xi_vel*sinh( (j/itr)*(Tsup/Tc) )+ px_mod;
        ytmp = (yi-py_mod)*cosh( (j/itr)*(Tsup/Tc) ) + Tc*yi_vel*sinh( (j/itr)*(Tsup/Tc) )+ py_mod;
        x_traj = [x_traj xtmp];
        y_traj = [y_traj ytmp];
        
    end
    
    COM_x=[COM_x;x_traj];
    COM_y=[COM_y;y_traj];
    
    xi_tmp = xi;
    yi_tmp = yi;
    xi = (xi-px_mod)*cosh( (Tsup/Tc) ) + Tc*xi_vel*sinh( (Tsup/Tc) )+ px_mod;
    yi = (yi-py_mod)*cosh( (Tsup/Tc) ) + Tc*yi_vel*sinh( (Tsup/Tc) )+ py_mod;
    xi_vel = (xi_tmp-px_mod)*sinh( (Tsup/Tc) )/Tc + xi_vel*cosh( (Tsup/Tc));
    yi_vel = (yi_tmp-py_mod)*sinh( (Tsup/Tc) )/Tc + yi_vel*cosh( (Tsup/Tc));
    
    
    %.....display or inverse kinematics.......
    
    x_traj;
    y_traj;
    
   plot(x_traj, y_traj);
   hold on;
   axis equal;
   pause;
    
    
    
   
   
      
    
   
    n = n+1;
    T = T + Tsup;
    
    px = px + sx(n);
    py = py - ((-1)^n)*sy(n);
    
   
    
    if(i==length(sx))
       plot(px,py,'*');
       plot(px,py,'o');
       break;
   end
    
    x = sx(n+1)/2;
    y = ((-1)^n) * sy(n+1)/2;
    
    C = cosh(Tsup/Tc);
    S = sinh(Tsup/Tc);
    
    x_vel = x*(C+1)/(Tc*S);
    y_vel = y*(C-1)/(Tc*S);
    
    x_d = px + x;
    x_vel_d  = x_vel;
    
    y_d = py + y;
    y_vel_d  = y_vel;
    
    
    a = 50;
    b = 1;
    D = a*(C-1)^2 + b* (S/Tc)^2;
    px_mod = -a*(C-1)*(x_d - C*xi - Tc*S*xi_vel)/D  - b*S*(x_vel_d - C*xi_vel - (S*xi)/Tc )/(Tc*D);
    
    py_mod = -a*(C-1)*(y_d - C*yi - Tc*S*yi_vel)/D  - b*S*(y_vel_d - C*yi_vel - (S*yi)/Tc )/(Tc*D);
    
end





    
