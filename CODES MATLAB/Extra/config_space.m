
global robot
global base
theta = zeros(1,28);

if base == 'leg'
    %Names and Ids for leg base
    r_arm = 16;
    l_arm = 21;
    l_leg = 28;
    from = 1;
    adj = 13;
elseif base == 'hip'
    %Names and Ids for hip base
    r_arm = 10;
    l_arm = 15;
    l_leg = 22;
    r_leg = 29;
    from = 1;
    
    theta_lim = [1500,850,1650,500,1600,4040,2150,2000,1380,1600,4040,2150,2000,1270,1500,500,2400,1500,600,1200,1500,1500,500,2400,1500,600,1200,1500]*360/4096;
    
    Idx = find_route(r_leg,from);
    len = length(Idx)
    l=1;
    pt1 = [];
    theta(7) = -20;
    theta(12) = 20;
    for i1 = 0:20:theta_lim(Idx(1))
        for i2 = 0:20:theta_lim(Idx(2))
            for i3 = 0:20:theta_lim(Idx(3))
                for i4 = 0:20:theta_lim(Idx(4))
                    for i5 = 0:20:theta_lim(Idx(5))
                        for i6 = 0:20:theta_lim(Idx(6))
                            for i7 = 0:20:theta_lim(Idx(7))
                                theta(Idx(1:len-1))= theta(Idx(1:len-1)) + [i1,i2,i3,i4,i5,i6,i7];
                                ForwKin(theta);
                                collision_data();
                                if(collision_check)
                                    pts(:,l) = robot.parts(Idx(len)).axis_loc;
                                    disp(l);
                                    l = l+1;
                                else
                                    fprintf('Not Done = %d\n',l);
                                end   
                            end
                        end
                    end
                end
            end
        end
    end
    
   save('config_pts2.mat','pts');
   disp('DONEEEEEEEEEEEEEEEE!!!!');
end


