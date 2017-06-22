function [v,w] = calVW(pid,cid,dq,fr)
global robot;
dq(29)=0;
if(fr == 1)
    vb = robot.parts(cid).v;
    wb = robot.parts(cid).w;
    v = vb + cross(wb,robot.parts(cid).R_mat*robot.parts(pid).br);
    w = wb + robot.parts(cid).R_mat*robot.parts(pid).ar*-dq(pid);
    
    robot.parts(pid).v = v;  
    robot.parts(pid).w = w;

    
else
    if(cid == 1)
        v = [0;0;0];
        w = robot.parts(cid).a*dq(cid);
        
        robot.parts(cid).v = v;
        robot.parts(cid).w = w;
    else
        vb = robot.parts(pid).v;
        wb = robot.parts(pid).w;
        v = vb + cross(wb,robot.parts(pid).R_mat*robot.parts(cid).b);
        w = wb + robot.parts(pid).R_mat*robot.parts(cid).a*dq(cid);
        
        robot.parts(cid).v = v;
        robot.parts(cid).w = w;
    end
end


end

