function [I_s] = Is(m,c,I)

c_hat = hat(c);

I_i = I + m * c_hat *c_hat';

I_s = [I_i          m*c_hat;
       m*c_hat'     m*eye(3)];
                    
end