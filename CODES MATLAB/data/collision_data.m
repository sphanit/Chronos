function [] = collision_data()

global robot;
global base;

if(base == 'rleg' | base == 'lleg' )
    
    %%r_leg to r_arm 
    robot.link(1).name = 'r_ankle';
    robot.link(1).type = 's';
    robot.link(1).id = 1;
    robot.link(1).u_id = 2;
    robot.link(1).l_id = 0;
    robot.link(1).pts = robot.draw(:,2);
    robot.link(1).radius = 36;
    
    robot.link(2).name = 'r_leg_low';
    robot.link(2).type = 'c';
    robot.link(2).id = 2;
    robot.link(2).u_id = 3;
    robot.link(2).l_id = 1;
    robot.link(2).pts = [robot.draw(:,3),robot.draw(:,4)];
    robot.link(2).radius = 30.5;
    
    robot.link(3).name = 'r_leg_high';
    robot.link(3).type = 'c';
    robot.link(3).id = 3;
    robot.link(3).u_id = 4;
    robot.link(3).l_id = 2;
    robot.link(3).pts = [robot.draw(:,4),robot.draw(:,5)];
    robot.link(3).radius = 30;
    
    robot.link(4).name = 'r_hip_low';
    robot.link(4).type = 'c';
    robot.link(4).id = 4;
    robot.link(4).u_id = 5;
    robot.link(4).l_id = 3;
    robot.link(4).pts = [robot.draw(:,5),robot.draw(:,6)];
    robot.link(4).radius = 24.5;
    
    robot.link(5).name = 'r_hip';
    robot.link(5).type = 'c';
    robot.link(5).id = 5;
    robot.link(5).u_id = 6;
    robot.link(5).l_id = 4;
    robot.link(5).pts = [robot.draw(:,6),robot.draw(:,7)];
    robot.link(5).radius = 24;
    
    robot.link(6).name = 'r_hip_high';
    robot.link(6).type = 'c';
    robot.link(6).id = 6;
    robot.link(6).u_id = 7;
    robot.link(6).l_id = 5;
    robot.link(6).pts = [robot.draw(:,7),robot.draw(:,8)];
    robot.link(6).radius = 31;    
    
    robot.link(7).name = 'abdomen';
    robot.link(7).type = 'c';
    robot.link(7).id = 7;
    robot.link(7).u_id = 8;
    robot.link(7).l_id = 6;
    robot.link(7).pts = [robot.draw(:,8),robot.draw(:,10)];
    robot.link(7).radius = 44.15;
    
    robot.link(8).name = 'torso';
    robot.link(8).type = 'c';
    robot.link(8).id = 8;
    robot.link(8).u_id = 9;
    robot.link(8).l_id = 7;
    robot.link(8).pts = [robot.draw(:,10),robot.draw(:,12)];
    robot.link(8).radius = 32;
    
    robot.link(9).name = 'r_chest';
    robot.link(9).type = 'c';
    robot.link(9).id = 9;
    robot.link(9).u_id = 10;
    robot.link(9).l_id = 8;
    robot.link(9).pts = [robot.draw(:,12),robot.draw(:,13)];
    robot.link(9).radius = 25;
    
    robot.link(10).name = 'r_shoulder';
    robot.link(10).type = 's';
    robot.link(10).id = 10;
    robot.link(10).u_id = 11;
    robot.link(10).l_id = 9;
    robot.link(10).pts = robot.draw(:,15);
    robot.link(10).radius = 28.2;
    
    robot.link(11).name = 'r_arm_high';
    robot.link(11).type = 'c';
    robot.link(11).id = 11;
    robot.link(11).u_id = 12;
    robot.link(11).l_id = 10;
    robot.link(11).pts = [robot.draw(:,15),robot.draw(:,16)];
    robot.link(11).radius = 23;
    
    robot.link(12).name = 'r_arm_low';
    robot.link(12).type = 'c';
    robot.link(12).id = 12;
    robot.link(12).u_id = 0;
    robot.link(12).l_id = 11;
    robot.link(12).pts = [robot.draw(:,16),robot.draw(:,17)];
    robot.link(12).radius = 23;
    
    %%chest to l_arm
    
    robot.link(13).name = 'l_chest';
    robot.link(13).type = 'c';
    robot.link(13).id = 13;
    robot.link(13).u_id = 14;
    robot.link(13).l_id = 8;
    robot.link(13).pts = [robot.draw(:,12),robot.draw(:,18)];
    robot.link(13).radius = 25;%31.2;
    
    robot.link(14).name = 'l_shoulder';
    robot.link(14).type = 's';
    robot.link(14).id = 14;
    robot.link(14).u_id = 15;
    robot.link(14).l_id = 13;
    robot.link(14).pts = robot.draw(:,20);
    robot.link(14).radius = 28.2;
    
    robot.link(15).name = 'l_arm_high';
    robot.link(15).type = 'c';
    robot.link(15).id = 15;
    robot.link(15).u_id = 16;
    robot.link(15).l_id = 14;
    robot.link(15).pts = [robot.draw(:,20),robot.draw(:,21)];
    robot.link(15).radius = 23;
    
    robot.link(16).name = 'l_arm_low';
    robot.link(16).type = 'c';
    robot.link(16).id = 16;
    robot.link(16).u_id = 0;
    robot.link(16).l_id = 15;
    robot.link(16).pts = [robot.draw(:,21),robot.draw(:,22)];
    robot.link(16).radius = 10;
    
    %%l_leg to l_arm
    robot.link(17).name = 'l_hip_high';
    robot.link(17).type = 'c';
    robot.link(17).id = 17;
    robot.link(17).u_id = 18;
    robot.link(17).l_id = 5;
    robot.link(17).pts = [robot.draw(:,8),robot.draw(:,23)];
    robot.link(17).radius = 31; 
    
    robot.link(18).name = 'l_hip';
    robot.link(18).type = 'c';
    robot.link(18).id = 18;
    robot.link(18).u_id = 19;
    robot.link(18).l_id = 17;
    robot.link(18).pts = [robot.draw(:,23),robot.draw(:,24)];
    robot.link(18).radius = 24;
    
    robot.link(19).name = 'l_hip_low';
    robot.link(19).type = 'c';
    robot.link(19).id = 19;
    robot.link(19).u_id = 20;
    robot.link(19).l_id = 18;
    robot.link(19).pts = [robot.draw(:,24),robot.draw(:,25)];
    robot.link(19).radius = 24.5;
    
    robot.link(20).name = 'l_leg_high';
    robot.link(20).type = 'c';
    robot.link(20).id = 20;
    robot.link(20).u_id = 21;
    robot.link(20).l_id = 19;
    robot.link(20).pts = [robot.draw(:,25),robot.draw(:,26)];
    robot.link(20).radius = 30;
    
    robot.link(21).name = 'l_leg_low';
    robot.link(21).type = 'c';
    robot.link(21).id = 21;
    robot.link(21).u_id = 22;
    robot.link(21).l_id = 20;
    robot.link(21).pts = [robot.draw(:,26),robot.draw(:,27)];
    robot.link(21).radius = 30.5;
    
    robot.link(22).name = 'l_ankle';
    robot.link(22).type = 's';
    robot.link(22).id = 22;
    robot.link(22).u_id = 0;
    robot.link(22).l_id = 21;
    robot.link(22).pts = robot.draw(:,28);
    robot.link(22).radius = 36;   
    
else
    %%r_leg to r_arm 
    robot.link(1).name = 'r_ankle';
    robot.link(1).type = 's';
    robot.link(1).id = 1;
    robot.link(1).u_id = 2;
    robot.link(1).l_id = 0;
    robot.link(1).pts = robot.draw(:,21);%done
    robot.link(1).radius = 36;
    
    robot.link(2).name = 'r_leg_low';
    robot.link(2).type = 'c';
    robot.link(2).id = 2;
    robot.link(2).u_id = 3;
    robot.link(2).l_id = 1;
    robot.link(2).pts = [robot.draw(:,19),robot.draw(:,20)];%done
    robot.link(2).radius = 30.5;
    
    robot.link(3).name = 'r_leg_high';
    robot.link(3).type = 'c';
    robot.link(3).id = 3;
    robot.link(3).u_id = 4;
    robot.link(3).l_id = 2;
    robot.link(3).pts = [robot.draw(:,18),robot.draw(:,19)];%done
    robot.link(3).radius = 30;
    
    robot.link(4).name = 'r_hip_low';
    robot.link(4).type = 'c';
    robot.link(4).id = 4;
    robot.link(4).u_id = 5;
    robot.link(4).l_id = 3;
    robot.link(4).pts = [robot.draw(:,17),robot.draw(:,18)];%done
    robot.link(4).radius = 24.5;
    
    robot.link(5).name = 'r_hip';
    robot.link(5).type = 'c';
    robot.link(5).id = 5;
    robot.link(5).u_id = 6;
    robot.link(5).l_id = 4;
    robot.link(5).pts = [robot.draw(:,16),robot.draw(:,17)];%doneo
    robot.link(5).radius = 24;
    
    robot.link(6).name = 'r_hip_high';
    robot.link(6).type = 'c';
    robot.link(6).id = 6;
    robot.link(6).u_id = 7;
    robot.link(6).l_id = 5;
    robot.link(6).pts = [robot.draw(:,1),robot.draw(:,16)];%done
    robot.link(6).radius = 31;    
    
    robot.link(7).name = 'abdomen';
    robot.link(7).type = 'c';
    robot.link(7).id = 7;
    robot.link(7).u_id = 8;
    robot.link(7).l_id = 6;
    robot.link(7).pts = [robot.draw(:,1),robot.draw(:,3)];%done
    robot.link(7).radius = 44.15;
    
    robot.link(8).name = 'torso';
    robot.link(8).type = 'c';
    robot.link(8).id = 8;
    robot.link(8).u_id = 9;
    robot.link(8).l_id = 7;
    robot.link(8).pts = [robot.draw(:,3),robot.draw(:,5)];%done
    robot.link(8).radius = 32;
    
    robot.link(9).name = 'r_chest';
    robot.link(9).type = 'c';
    robot.link(9).id = 9;
    robot.link(9).u_id = 10;
    robot.link(9).l_id = 8;
    robot.link(9).pts = [robot.draw(:,5),robot.draw(:,6)];%done
    robot.link(9).radius = 25;
    
    robot.link(10).name = 'r_shoulder';
    robot.link(10).type = 's';
    robot.link(10).id = 10;
    robot.link(10).u_id = 11;
    robot.link(10).l_id = 9;
    robot.link(10).pts = robot.draw(:,8);%done
    robot.link(10).radius = 28.2;
    
    robot.link(11).name = 'r_arm_high';
    robot.link(11).type = 'c';
    robot.link(11).id = 11;
    robot.link(11).u_id = 12;
    robot.link(11).l_id = 10;
    robot.link(11).pts = [robot.draw(:,8),robot.draw(:,9)];%done
    robot.link(11).radius = 23;
    
    robot.link(12).name = 'r_arm_low';
    robot.link(12).type = 'c';
    robot.link(12).id = 12;
    robot.link(12).u_id = 0;
    robot.link(12).l_id = 11;
    robot.link(12).pts = [robot.draw(:,9),robot.draw(:,10)];%done
    robot.link(12).radius = 23;
    
    %%chest to l_arm
    
    robot.link(13).name = 'l_chest';
    robot.link(13).type = 'c';
    robot.link(13).id = 13;
    robot.link(13).u_id = 14;
    robot.link(13).l_id = 8;
    robot.link(13).pts = [robot.draw(:,5),robot.draw(:,11)];%done
    robot.link(13).radius = 25;%31.2;
    
    robot.link(14).name = 'l_shoulder';
    robot.link(14).type = 's';
    robot.link(14).id = 14;
    robot.link(14).u_id = 15;
    robot.link(14).l_id = 13;
    robot.link(14).pts = robot.draw(:,13);%done
    robot.link(14).radius = 28.2;
    
    robot.link(15).name = 'l_arm_high';
    robot.link(15).type = 'c';
    robot.link(15).id = 15;
    robot.link(15).u_id = 16;
    robot.link(15).l_id = 14;
    robot.link(15).pts = [robot.draw(:,13),robot.draw(:,14)];%done
    robot.link(15).radius = 23;
    
    robot.link(16).name = 'l_arm_low';
    robot.link(16).type = 'c';
    robot.link(16).id = 16;
    robot.link(16).u_id = 0;
    robot.link(16).l_id = 15;
    robot.link(16).pts = [robot.draw(:,14),robot.draw(:,15)];%done
    robot.link(16).radius = 23;
    
    %%l_leg to l_arm
    robot.link(17).name = 'l_hip_high';
    robot.link(17).type = 'c';
    robot.link(17).id = 17;
    robot.link(17).u_id = 18;
    robot.link(17).l_id = 5;
    robot.link(17).pts = [robot.draw(:,1),robot.draw(:,23)];
    robot.link(17).radius = 31; 
    
    robot.link(18).name = 'l_hip';
    robot.link(18).type = 'c';
    robot.link(18).id = 18;
    robot.link(18).u_id = 19;
    robot.link(18).l_id = 17;
    robot.link(18).pts = [robot.draw(:,23),robot.draw(:,24)];
    robot.link(18).radius = 24;
    
    robot.link(19).name = 'l_hip_low';
    robot.link(19).type = 'c';
    robot.link(19).id = 19;
    robot.link(19).u_id = 20;
    robot.link(19).l_id = 18;
    robot.link(19).pts = [robot.draw(:,24),robot.draw(:,25)];
    robot.link(19).radius = 24.5;
    
    robot.link(20).name = 'l_leg_high';
    robot.link(20).type = 'c';
    robot.link(20).id = 20;
    robot.link(20).u_id = 21;
    robot.link(20).l_id = 19;
    robot.link(20).pts = [robot.draw(:,25),robot.draw(:,26)];
    robot.link(20).radius = 30;
    
    robot.link(21).name = 'l_leg_low';
    robot.link(21).type = 'c';
    robot.link(21).id = 21;
    robot.link(21).u_id = 22;
    robot.link(21).l_id = 20;
    robot.link(21).pts = [robot.draw(:,26),robot.draw(:,27)];
    robot.link(21).radius = 30.5;
    
    robot.link(22).name = 'l_ankle';
    robot.link(22).type = 's';
    robot.link(22).id = 22;
    robot.link(22).u_id = 0;
    robot.link(22).l_id = 21;
    robot.link(22).pts = robot.draw(:,28);
    robot.link(22).radius = 36;   
end
end

