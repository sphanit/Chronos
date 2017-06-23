%initialization of various model parts

function [robot] = make_model()
global base
if(base == 'lleg' | base == 'rleg')
    for i=1:16
        robot.parts(i).Id = i;
        robot.parts(i).P_Id = i-1;
        if(i==16)
            robot.parts(i).C_Id = 0;
        else
            robot.parts(i).C_Id = i+1;
        end
        robot.parts(i).axis_loc = [0,0,0];
        robot.parts(i).joint_loc = [0,0,0];
        robot.parts(i).Z_w =[0,0,1];
        robot.parts(i).R_mat = eye(3);
        robot.parts(i).b =[0,0,0];
        robot.parts(i).a =[0,0,1];
        robot.parts(i).ar =[0,0,1];
        robot.parts(i).br =[0,0,1];
        robot.parts(i).mass = 0;
        robot.parts(i).I = eye(3);
        robot.parts(i).com_g =[0,0,0];
        robot.parts(i).com_lg =[0,0,0];
        robot.parts(i).com_l =[0,0,0];
        robot.parts(i).v =[0;0;0];
        robot.parts(i).w =[0;0;0];
        robot.parts(i).P =[0,0,0];
        robot.parts(i).L =[0,0,0];
        robot.parts(i).dq = 0;
        robot.parts(i).ddq = 0;
        
    end
    
    for i=17:21
        robot.parts(i).Id = i;
        if(i==17)
            robot.parts(i).P_Id = 11;
        else
            robot.parts(i).P_Id = i-1;
        end
        
        if(i==21)
            robot.parts(i).C_Id = 0;
        else
            robot.parts(i).C_Id = i+1;
        end
        
        robot.parts(i).axis_loc = [0,0,0];
        robot.parts(i).joint_loc = [0,0,0];
        robot.parts(i).Z_w =[0,0,1];
        robot.parts(i).R_mat = eye(3);
        robot.parts(i).b =[0,0,0];
        robot.parts(i).a =[0,0,1];
        robot.parts(i).ar =[0,0,1];
        robot.parts(i).br =[0,0,1];
        robot.parts(i).mass = 0;
        robot.parts(i).I = eye(3);
        robot.parts(i).com_g =[0,0,0];
        robot.parts(i).com_lg =[0,0,0];
        robot.parts(i).com_l =[0,0,0];
        robot.parts(i).v =[0;0;0];
        robot.parts(i).w =[0;0;0];
        robot.parts(i).P =[0,0,0];
        robot.parts(i).L =[0,0,0];
        robot.parts(i).dq = 0;
        robot.parts(i).ddq = 0;
    end
    
    for i=22:28
        robot.parts(i).Id = i;
        if(i==22)
            robot.parts(i).P_Id = 7;
        else
            robot.parts(i).P_Id = i-1;
        end
        
        if(i==28)
            robot.parts(i).C_Id = 0;
        else
            robot.parts(i).C_Id = i+1;
        end
        
        robot.parts(i).axis_loc = [0,0,0];
        robot.parts(i).joint_loc = [0,0,0];
        robot.parts(i).Z_w =[0,0,1];
        robot.parts(i).R_mat = eye(3);
        robot.parts(i).b =[0,0,0];
        robot.parts(i).a =[0,0,1];
        robot.parts(i).ar =[0,0,1];
        robot.parts(i).br =[0,0,1];
        robot.parts(i).mass = 0;
        robot.parts(i).I = eye(3);
        robot.parts(i).com_g =[0,0,0];
        robot.parts(i).com_lg =[0,0,0];
        robot.parts(i).com_l =[0,0,0];
        robot.parts(i).v =[0;0;0];
        robot.parts(i).w =[0;0;0];
        robot.parts(i).P =[0,0,0];
        robot.parts(i).L =[0,0,0];
        robot.parts(i).dq = 0;
        robot.parts(i).ddq = 0;
    end
    
elseif(base == 'hips')
    for i=1:10
        robot.parts(i).Id = i;
        robot.parts(i).P_Id = i-1;
        if(i==10)
            robot.parts(i).C_Id = 0;
        else
            robot.parts(i).C_Id = i+1;
        end
        robot.parts(i).axis_loc = [0,0,0];
        robot.parts(i).joint_loc = [0,0,0];
        robot.parts(i).Z_w =[0,0,1];
        robot.parts(i).R_mat = eye(3);
        robot.parts(i).b =[0,0,0];
        robot.parts(i).a =[0,0,1];
        robot.parts(i).ar =[0,0,1];
        robot.parts(i).br =[0,0,1];
        robot.parts(i).mass = 0;
        robot.parts(i).I = eye(3);
        robot.parts(i).com_g =[0,0,0];
        robot.parts(i).com_lg =[0,0,0];
        robot.parts(i).com_l =[0,0,0];
        robot.parts(i).v =[0;0;0];
        robot.parts(i).w =[0;0;0];
        robot.parts(i).P =[0,0,0];
        robot.parts(i).L =[0,0,0];
        robot.parts(i).dq = 0;
        robot.parts(i).ddq = 0;
    end
    
    for i=11:15
        robot.parts(i).Id = i;
        if(i==11)
            robot.parts(i).P_Id = 5;
        else
            robot.parts(i).P_Id = i-1;
        end
        
        if(i==15)
            robot.parts(i).C_Id = 0;
        else
            robot.parts(i).C_Id = i+1;
        end
        
        robot.parts(i).axis_loc = [0,0,0];
        robot.parts(i).joint_loc = [0,0,0];
        robot.parts(i).Z_w =[0,0,1];
        robot.parts(i).R_mat = eye(3);
        robot.parts(i).b =[0,0,0];
        robot.parts(i).a =[0,0,1];
        robot.parts(i).ar =[0,0,1];
        robot.parts(i).br =[0,0,1];
        robot.parts(i).mass = 0;
        robot.parts(i).I = eye(3);
        robot.parts(i).com_g =[0,0,0];
        robot.parts(i).com_lg =[0,0,0];
        robot.parts(i).com_l =[0,0,0];
        robot.parts(i).v =[0;0;0];
        robot.parts(i).w =[0;0;0];
        robot.parts(i).P =[0,0,0];
        robot.parts(i).L =[0,0,0];
        robot.parts(i).dq = 0;
        robot.parts(i).ddq = 0;
    end
    
    for i=16:22
        robot.parts(i).Id = i;
        if(i==16)
            robot.parts(i).P_Id = 1;
        else
            robot.parts(i).P_Id = i-1;
        end
        
        if(i==22)
            robot.parts(i).C_Id = 0;
        else
            robot.parts(i).C_Id = i+1;
        end
        
        robot.parts(i).axis_loc = [0,0,0];
        robot.parts(i).joint_loc = [0,0,0];
        robot.parts(i).Z_w =[0,0,1];
        robot.parts(i).R_mat = eye(3);
        robot.parts(i).b =[0,0,0];
        robot.parts(i).a =[0,0,1];
        robot.parts(i).ar =[0,0,1];
        robot.parts(i).br =[0,0,1];
        robot.parts(i).mass = 0;
        robot.parts(i).I = eye(3);
        robot.parts(i).com_g =[0,0,0];
        robot.parts(i).com_lg =[0,0,0];
        robot.parts(i).com_l =[0,0,0];
        robot.parts(i).v =[0;0;0];
        robot.parts(i).w =[0;0;0];
        robot.parts(i).P =[0,0,0];
        robot.parts(i).L =[0,0,0];
        robot.parts(i).dq = 0;
        robot.parts(i).ddq = 0;
    end
    
    for i=23:29
        robot.parts(i).Id = i;
        if(i==23)
            robot.parts(i).P_Id = 1;
        else
            robot.parts(i).P_Id = i-1;
        end
        
        if(i==29)
            robot.parts(i).C_Id = 0;
        else
            robot.parts(i).C_Id = i+1;
        end
        
        robot.parts(i).axis_loc = [0,0,0];
        robot.parts(i).joint_loc = [0,0,0];
        robot.parts(i).Z_w =[0,0,1];
        robot.parts(i).R_mat = eye(3);
        robot.parts(i).b =[0,0,0];
        robot.parts(i).a =[0,0,1];
        robot.parts(i).ar =[0,0,1];
        robot.parts(i).br =[0,0,1];
        robot.parts(i).mass = 0;
        robot.parts(i).I = eye(3);
        robot.parts(i).com_g =[0,0,0];
        robot.parts(i).com_lg =[0,0,0];
        robot.parts(i).com_l =[0,0,0];
        robot.parts(i).v =[0;0;0];
        robot.parts(i).w =[0;0;0];
        robot.parts(i).P =[0,0,0];
        robot.parts(i).L =[0,0,0];
        robot.parts(i).dq = 0;
        robot.parts(i).ddq = 0;
    end
    
end
end