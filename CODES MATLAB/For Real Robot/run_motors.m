% % 1  -> (-)22
% % 2  -> (-)23
% % 3  -> (+)24
% % 4  -> (+)25
% % 5  -> (-)26
% % 6  -> (-)27
% % 7  -> (-)6
% % 8  -> (-)5
% % 9  -> (-)4
% % 10 -> (-)3
% % 11 -> (+)2
% % 12 -> (-)1
% % 13 -> (-)7
% % 14 -> (-)8
% % 15 -> (-)9
% % 16 -> (-)10
% % 17 -> (+)11
% % 18 -> (+)17
% % 19 -> (+)18
% % 20 -> (+)19
% % 21 -> (-)20
% % 22 -> (-)12
% % 23 -> (+)13
% % 24 -> (+)14
% % 25 -> (+)15


clc;
clear all;
close all;

lib_name = '';

if strcmp(computer, 'PCWIN')
    lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
    lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
    lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
    lib_name = 'libdxl_x64_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_bulk_read.h');
end

%Load Angle data
load('left_arm_manip.mat');

% Control table address
ADDR_MX_TORQUE_ENABLE           = 24;           % Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION           = 30;
ADDR_MX_PRESENT_POSITION        = 36;
ADDR_MX_MOVING                  = 46;

% Data Byte Length
LEN_MX_GOAL_POSITION            = 2;
LEN_MX_PRESENT_POSITION         = 2;
LEN_MX_MOVING                   = 1;

% Protocol version
PROTOCOL_VERSION                = 1.0;          % See which protocol version is used in the Dynamixel

% Default setting
idx                             = [11,12,13,14,15,16,21,22,23,24,25,26,31,32,33,34,35,41,42,43,44,51,52,53,54];
DXL_IDs                         = idx;
BAUDRATE                        = 57142;
DEVICENAME                      = 'COM19';       % Check which port is being used on your controller
% ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

TORQUE_ENABLE                   = 1;            % Value for enabling the torque
TORQUE_DISABLE                  = 0;            % Value for disabling the torque

ESC_CHARACTER                   = 'e';          % Key for escaping loop

COMM_SUCCESS                    = 0;            % Communication Success result value
COMM_TX_FAIL                    = -1001;        % Communication Tx Failed

DXL_MINIMUM_POSITION_VALUE      = (180-90)*(4096/360);          % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE      = (150+90)*(4096/360);         % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD     = 10;           % Dynamixel moving status threshold


% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

index = 1;
dxl_comm_result = COMM_TX_FAIL;             % Communication result


% For Stable Standing                       % Goal position
dxl_goal_position(1,1:25) = (180)*(4096/360);          
dxl_goal_position(1,4)    = (180+20)*(4096/360);
dxl_goal_position(1,5)    = (180-20)*(4096/360);
dxl_goal_position(1,10)   = (180-20)*(4096/360);
dxl_goal_position(1,11)   = (180+20)*(4096/360);

% Angle map from Simulation

[m_ang] = ang_map(q);
dxl_goal_position = m_ang;

dxl_error = 0;                              % Dynamixel error
dxl_present_position = 0;                   % Present position


% Open port
if (openPort(port_num))
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
end


% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    closePort(port_num);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

% Try to ping the Dynamixel
% Get Dynamixel model number
count = 0;
for i = 1:length(idx)
    dxl_model_number = pingGetModelNum(port_num, PROTOCOL_VERSION, DXL_IDs(i));
    
    if getLastTxRxResult(port_num, PROTOCOL_VERSION) == COMM_SUCCESS
        printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
        count = count + 1;
        fprintf('[ID:%03d] ping Succeeded. Dynamixel model number : %d\n', DXL_IDs(i), dxl_model_number);
    else
        fprintf('[ID:%03d] ping Unsuccessful.\n', DXL_IDs(i));
    end
    
end

fprintf('\nNo: of Motors recognized = %d\n\n',count);

if(count == 25)
    for i = 1:length(idx)
        % Enable Dynamixel Torque
        write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDs(i), ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
        if getLastTxRxResult(port_num, PROTOCOL_VERSION) ~= COMM_SUCCESS
            printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
        elseif getLastRxPacketError(port_num, PROTOCOL_VERSION) ~= 0
            printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num, PROTOCOL_VERSION));
        else
            fprintf('Torque enabled for [ID:%03d]\n',DXL_IDs(i));
        end
    end
    
    m_an = 1;
    
    while (m_an ~= 41)
        if(mod(m_an,10) == 0)
            in = input('Press any key to continue!');
            disp(m_an);
        end
        for i = 1:length(idx)
            % Write goal position
            write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDs(i), ADDR_MX_GOAL_POSITION, dxl_goal_position(i,m_an));
            
%             if getLastTxRxResult(port_num, PROTOCOL_VERSION) ~= COMM_SUCCESS
%                 printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
%             elseif getLastRxPacketError(port_num, PROTOCOL_VERSION) ~= 0
%                 printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num, PROTOCOL_VERSION));
%             end
        end
        m_an = m_an + 1;
    end
    
    pause
    
    m_an = 40;
    while (m_an ~= 0)
        if(mod(m_an,10) == 0)
            in = input('Press any key to continue!');
            disp(m_an);
        end        
        for i = 1:length(idx)
            % Write goal position
            write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDs(i), ADDR_MX_GOAL_POSITION, dxl_goal_position(i,m_an));
            
%             if getLastTxRxResult(port_num, PROTOCOL_VERSION) ~= COMM_SUCCESS
%                 printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
%             elseif getLastRxPacketError(port_num, PROTOCOL_VERSION) ~= 0
%                 printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num, PROTOCOL_VERSION));
%             end
        end
        m_an = m_an - 1;
    end
    
    
    if(input('Press any key to continue! (or input d to disable torque!)\n','s') == 'd')
        for i = 1:length(idx)
            % Disable Dynamixel Torque
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDs(i), ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
            if getLastTxRxResult(port_num, PROTOCOL_VERSION) ~= COMM_SUCCESS
                printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
            elseif getLastRxPacketError(port_num, PROTOCOL_VERSION) ~= 0
                printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num, PROTOCOL_VERSION));
            end
        end
    end
    
else
    % Close port
    closePort(port_num);
    
    % Unload Library
    unloadlibrary(lib_name);
    
    return;
end

% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);

