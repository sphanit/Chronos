function [m_ang] = ang_map(ang)
%Mapping
m_ang(1,:)  = -ang(22,:);
m_ang(2,:)  = -ang(23,:);
m_ang(3,:)  = +ang(24,:);
m_ang(4,:)  = +ang(25,:);
m_ang(5,:)  = +ang(26,:);
m_ang(6,:)  = -ang(27,:);
m_ang(7,:)  = +ang(6,:);
m_ang(8,:)  = +ang(5,:);
m_ang(9,:)  = +ang(4,:);
m_ang(10,:) = +ang(3,:);
m_ang(11,:) = +ang(2,:);
m_ang(12,:) = +ang(1,:);
m_ang(13,:) = -ang(7,:);
m_ang(14,:) = +ang(8,:);
m_ang(15,:) = +ang(9,:);
m_ang(16,:) = -ang(10,:);
m_ang(17,:) = -ang(11,:);
m_ang(18,:) = +ang(17,:);
m_ang(19,:) = -ang(18,:);
m_ang(20,:) = -ang(19,:);
m_ang(21,:) = -ang(20,:);
m_ang(22,:) = -ang(12,:);
m_ang(23,:) = -ang(13,:);
m_ang(24,:) = -ang(14,:);
m_ang(25,:) = +ang(15,:);

% For single step
% m_ang(1,:) = m_ang(1,:);
% m_ang(7,:) = m_ang(1,:);
% m_ang(17,:) = (180-20)*(4096/360);
% m_ang(16,:) = (180-5)*(4096/360);
% m_ang(22,:) = (180+10)*(4096/360);
% m_ang(23,:) = (180+35)*(4096/360);
% m_ang(13,:) = (180-15)*(4096/360);
% tmp = 5;
% m_ang(12,:) = (180+tmp)*(4096/360);


% Conversion
m_ang(:,:) = (180+m_ang(:,:))*(4096/360);


% For stable double support phase

% m_ang(4,:) = (180+20)*(4096/360);
% m_ang(5,:) = (180-20)*(4096/360);
% m_ang(10,:) = (180-20)*(4096/360);
% m_ang(11,:) = (180+20)*(4096/360);

end

