function [dq,v,w] = ForwVel2(q,vb,wb)
global robot;
global base;
global change;

dq = diff(q,1,2);
dq(29,99) = 0;

v(:,1,1) = [0;0;0];
w(:,1,1) = [0;0;0];

v(:,29,100) = [0;0;0];
w(:,29,100) = [0;0;0];

v(:,:,1) = vb(:,:);
w(:,:,1) = wb(:,:);

if (base == 'hip')
    
    for l = 2:length(q)
        
        if(change == 1)
            for i = 28:-1:23
                tmp = robot.parts(i).C_Id;
                v(:,i,l) = v(:,tmp,l) + cross(w(:,tmp,l),robot.parts(tmp).R_mat*robot.parts(tmp).br);
                w(:,i,l) = w(:,tmp,l) + robot.parts(tmp).R_mat*robot.parts(tmp).ar*-dq(i,l-1);
            end
            
            i = 1;
            tmp = 23;
            v(:,i,l) =  v(:,tmp,l) + cross(w(:,tmp,l),robot.parts(tmp).R_mat*robot.parts(tmp).br);
            w(:,i,l) =  w(:,tmp,l) + robot.parts(tmp).R_mat*robot.parts(tmp).ar*-dq(i,l-1);
            
            for i = 2:15
                tmp = robot.parts(i).P_Id;
                v(:,i,l) = v(:,tmp,l) + cross(w(:,tmp,l),robot.parts(tmp).R_mat*robot.parts(tmp).b);
                w(:,i,l) = w(:,tmp,l) + robot.parts(tmp).R_mat*robot.parts(tmp).a*dq(i,l-1);
            end
            
            for i = 16:22
                tmp = robot.parts(i).P_Id;
                v(:,i,l) = v(:,tmp,l) + cross(w(:,tmp,l),robot.parts(tmp).R_mat*robot.parts(tmp).b);
                w(:,i,l) = w(:,tmp,l) + robot.parts(tmp).R_mat*robot.parts(tmp).a*dq(i,l-1);
            end 
        end
        
        if(change == 0)
            for i = 21:-1:16
                tmp = robot.parts(i).C_Id;
                v(:,i,l) = v(:,tmp,l) + cross(w(:,tmp,l),robot.parts(tmp).R_mat*robot.parts(tmp).br);
                w(:,i,l) = w(:,tmp,l) + robot.parts(tmp).R_mat*robot.parts(tmp).ar*-dq(i,l-1);
            end
            
            i = 1;
            tmp = 16;
            v(:,i,l) =  v(:,tmp,l) + cross(w(:,tmp,l),robot.parts(tmp).R_mat*robot.parts(tmp).br);
            w(:,i,l) =  w(:,tmp,l) + robot.parts(tmp).R_mat*robot.parts(tmp).ar*-dq(i,l-1);
            
            for i = 2:15
                tmp = robot.parts(i).P_Id;
                v(:,i,l) = v(:,tmp,l) + cross(w(:,tmp,l),robot.parts(tmp).R_mat*robot.parts(tmp).b);
                w(:,i,l) = w(:,tmp,l) + robot.parts(tmp).R_mat*robot.parts(tmp).a*dq(i,l-1);
            end
            
            for i = 23:29
                tmp = robot.parts(i).P_Id;
                v(:,i,l) = v(:,tmp,l) + cross(w(:,tmp,l),robot.parts(tmp).R_mat*robot.parts(tmp).b);
                w(:,i,l) = w(:,tmp,l) + robot.parts(tmp).R_mat*robot.parts(tmp).a*dq(i,l-1);
            end                       
        end
    end
    
else
    %Don't know what to do
end
            change = ~change;
end

% t2(:,:) = v2{2}(:,27,:)
% t1(:,:) = v1{2}(:,27,:)
% t3(:,:) = v1{3}(:,27,:)
% hold on,plot((-[t1(3,:),t2(3,:),t3(3,:)]),'k.-')
% hold on,plot((-[t1(2,:),t2(2,:),t3(2,:)]),'b.-')
% hold on,plot((-[t1(1,:),t2(1,:),t3(1,:)]),'r.-')
% hold on,plot((-[t1(3,:),t2(3,:),t3(3,:)]),'k.-')
% hold on, plot(-diff(z));
% hold on, plot(-diff(z(1:200)));
% hold on,plot((-[t1(3,:),t2(3,:),t3(3,:)]),'k.-')
% hold on, plot(-diff(x(1:200)));
% hold on,plot((-[t1(3,:),t2(3,:),t3(3,:)]),'k.-')
% hold on, plot(-diff(x(1:200)));
% hold on,plot(([t1(1,:),t2(1,:),t3(1,:)]),'r.-')
% hold on,plot((-[t1(1,:),t2(1,:),t3(1,:)]),'k.-')
% hold on, plot(-diff(x(1:200)));
% hold on, plot(-diff(x(1:300)),'.');
% hold on,plot((-[t1(1,:),t2(1,:),t3(1,:)]),'k.-')