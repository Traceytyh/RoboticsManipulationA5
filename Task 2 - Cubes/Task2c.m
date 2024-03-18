% Read the position of the dynamixel horn with the torque off
% The code executes for a given amount of time then terminates
 
 
clc;
clear all;

lib_name = '';
 
if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end
 
%% ---- Control Table Addresses ---- %%
 
ADDR_PRO_TORQUE_ENABLE       = 64;           % Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION       = 116; 
ADDR_PRO_PRESENT_POSITION    = 132; 
ADDR_PRO_OPERATING_MODE      = 11;    % 1BYTE
 
%% ---- Other Settings ---- %%
 
% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel
 
% Default setting
DXL_ID1                     = 11; 
DXL_ID2                     = 12; 
DXL_ID3                     = 13;
DXL_ID4                     = 14;
DXL_ID5                     = 15;
BAUDRATE                    = 115200;
DEVICENAME                  = 'COM9';      % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = -150000;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 150000;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold
 
ESC_CHARACTER               = 'e';          % Key for escaping loop
 
COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;       
% Communication Tx Failed
 
%% ------------------ %%
 
% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);
 
% Initialize PacketHandler Structs
packetHandler();
 
index = 1;
dxl_comm_result = COMM_TX_FAIL;           % Communication result
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];         % Goal position
 
dxl_error = 0;                              % Dynamixel error
dxl_present_position = 0;                   % Present position

ADDR_GOAL_CURRENT            = 102; 
 
% ----- SET MOTION LIMITS ----------- %
ADDR_MAX_POS = 48;
ADDR_MIN_POS = 52;
%MAX_POS = 3400;
%MIN_POS = 600;
DXL_ID1_MIN_POS = 0000;  % Min and max for DXL_ID1
DXL_ID1_MAX_POS = 4000;
DXL_ID2_MIN_POS = 0750;  % Min and max for DXL_ID2
DXL_ID2_MAX_POS = 3310;
DXL_ID3_MIN_POS = 0690;  % Min and max for DXL_ID3
DXL_ID3_MAX_POS = 3050;
DXL_ID4_MIN_POS = 0837;  % Min and max for DXL_ID4
DXL_ID4_MAX_POS = 3463;
DXL_ID5_MIN_POS = 3530;  % Min and max for DXL_ID5
DXL_ID5_MAX_POS = 6714;

% Theta limits
THETA1_MIN = -180;
THETA1_MAX =  180;
THETA3_MIN = -193;
THETA3_MAX =   32;
THETA4_MIN =  -39;
THETA4_MAX =  169;
THETA5_MIN = -106;
THETA5_MAX =  125;


% ---------------------------------- %
% Open port
if (openPort(port_num))
    fprintf('Port Open\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port\n');
    input('Press any key to terminate...\n');
    return;
end
 
 
% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
 
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end
 
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result)); %print result
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
 % Set max position limit
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MAX_POS, DXL_ID1_MAX_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_MAX_POS, DXL_ID2_MAX_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_MAX_POS, DXL_ID3_MAX_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_MAX_POS, DXL_ID4_MAX_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_MAX_POS, DXL_ID5_MAX_POS);
% Set min position limit
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MIN_POS, DXL_ID1_MIN_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_MIN_POS, DXL_ID2_MIN_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_MIN_POS, DXL_ID3_MIN_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_MIN_POS, DXL_ID4_MIN_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_MIN_POS, DXL_ID5_MIN_POS);
 
% Put actuator into Position Control Mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_OPERATING_MODE, 5);
write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_GOAL_CURRENT, 150);
 
% Enable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_TORQUE_ENABLE, 1);
    
dxl_present_position5 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_PRESENT_POSITION);
closed = 2500;
open = 1200;

% position = [x, y, cube stack level]
% Holder positions 1, 2, 3 - grip horizontally
holder_pos1 = [ 70, -195, 1];   % r = 215
holder_pos2 = [217,    0, 1];   % r = 226
holder_pos3 = [143,  144, 1];   % r = 210
%holder_pos4 = [132, -131, 0];   % r = 185
holder_pos4 = [127, -126, 0];
%holder_pos4hor = [122, -121, 0];
holder_pos5 = [110,   -5, 0];   % r = 100
holder_pos6 = [  0,  115, 0];   % r = 100
rotate_pos4 = [115, -114, 0];

%check_radius(holder_pos3)

% Old (rotate cube at position 4)
% move_to(holder_pos4, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
% holder_pos4(3) = pick(holder_pos4, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
% move_to(rotate_pos4, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
% move_to(holder_pos4, 1, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
% holder_pos4(3) = place(holder_pos4, 1, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);

% New rotation using rotate()
% Should pick cube at position 1, move to 4 for rotation, then place at 6
%move_to(holder_pos1, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
%rotate(holder_pos1, holder_pos4, 2, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
%holder_pos4(3) = holder_pos4(3) + 1;
%holder_pos4(3) = pick(holder_pos4, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
%move_to(holder_pos6, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
%holder_pos6(3) = place(holder_pos6, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
% Should pick cube at position 2, move to 4 for rotation, then stack onto 6
move_to(holder_pos2, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
rotate(holder_pos2, holder_pos4, 3, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
holder_pos4(3) = holder_pos4(3) + 1;
holder_pos4(3) = pick(holder_pos4, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
move_to(holder_pos6, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
holder_pos6(3) = place(holder_pos6, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
% Should pick cube at position 3, move to 4 for rotation, then stack onto 6
%move_to(holder_pos3, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
%rotate(holder_pos3, holder_pos4, 2, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
%holder_pos4(3) = holder_pos4(3) + 1;
%holder_pos4(3) = pick(holder_pos4, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
%move_to(holder_pos6, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
%holder_pos6(3) = place(holder_pos6, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);

% Disable Dynamixel Torque
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_TORQUE_ENABLE, 0);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_TORQUE_ENABLE, 0);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_TORQUE_ENABLE, 0);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_TORQUE_ENABLE, 0);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_TORQUE_ENABLE, 0);

% Close port
closePort(port_num);
fprintf('Port Closed \n');
 
% Unload Library
unloadlibrary(lib_name);
 
close all;
%clear all;

function [x, y, z, angle] = currentangles(port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION)
    position1 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_PRESENT_POSITION);
    position3 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_PRESENT_POSITION);
    position4 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_PRESENT_POSITION);
    position5 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_PRESENT_POSITION);
    thetafix = asin(0.024/0.13)/pi*180;
    theta1 = (position1 - 2048)/2048*180;
    theta3 = (position3 - 2048)/2048*180 - 90 + thetafix;
    theta4 = (position4 - 2048)/2048*180 + 90 - thetafix;
    theta5 = (position5 - 2048)/2048*180;
    r = 130*cosd(theta3)+124*cosd(-theta3-theta4)+126*cosd(-theta3-theta4-theta5);
    x = r*cosd(theta1);
    y = r*sind(theta1);
    z = 77-130*sind(theta3)-124*sind(theta3+theta4)-126*sind(theta3+theta4+theta5);
    angle = theta3+theta4+theta5;
end

function [position1, position2, position3, position4] = IK(x_desired, y_desired, z_desired, angle_desired)
    a3 = 130;
    a4 = 124;
    a5 = 126;
    d2 = 77;
    THETA1_MIN = -180;
    THETA1_MAX =  180;
    THETA3_MIN = -193;
    THETA3_MAX =   32;
    THETA4_MIN =  -39;
    THETA4_MAX =  169;
    THETA5_MIN = -106;
    THETA5_MAX =  125;
    r_desired = sqrt(y_desired.^2 + x_desired.^2);
    [~,icol] = size(x_desired);
    theta1 = zeros(1, icol);
    theta3 = zeros(1, icol);
    theta4 = zeros(1, icol);
    theta5 = zeros(1, icol);
    for i = 1:icol
        theta1(i) = atan2d(y_desired(i), x_desired(i));
        r3 = r_desired(i) - a5*cosd(angle_desired(i));
        z3 = z_desired(i) + a5*sind(angle_desired(i)) - d2;
        h3 = sqrt(r3^2 + z3^2);
        if (a3+a4) >= h3
            %angle a and B
            a = acosd((a3^2 + a4^2 - h3^2 )/(2*a3*a4));
            B = acosd((a3^2 + h3^2 - a4^2 )/(2*a3*h3));
            %joint angles elbow-down
            J1a = -(atan2d(z3,r3)+B);
            J2a = 180-a;
            J3a = angle_desired(i) - J1a -J2a;
            %joint angles elbow-up
            J1b = -(atan2d(z3,r3)-B);
            J2b = a-180;
            J3b = angle_desired(i) - J1b - J2b;
            if (THETA3_MIN < J1b) && (J1b < THETA3_MAX) && (THETA4_MIN < J2b) && (J2b < THETA4_MAX) && (THETA5_MIN < J3b) && (J3b < THETA5_MAX) 
                theta3(i) = 1*J1b;
                theta4(i) = 1*J2b;
                theta5(i) = 1*J3b;
            else
                theta3(i) = 1*J1a;
                theta4(i) = 1*J2a;
                theta5(i) = 1*J3a;
            end
        else
            disp("out of bound");
            if i ~=1
               theta3(i) = theta3(i - 1);
               theta4(i) = theta4(i - 1);
               theta5(i) = theta5(i - 1);
            end
        end
    end
    thetafix = asin(0.024/0.13)/pi*180;
    position1 = theta1./180 .* 2048 + 2048;
    position2 = (theta3 - thetafix + 90)./180 .* 2048 + 2048;
    position3 = (theta4+ thetafix - 90)./180 .* 2048 + 2048;
    position4 = theta5./180 .* 2048 + 2048;
end

function rad = check_radius(position) % return 1 if radius > 207, 0 if otherwise
    if(sqrt((position(1)^2) + (position(2)^2)) > 200)
        rad = 1;
    else
        rad = 0;
    end
end

function [xTraj, yTraj, zTraj, angleTraj] = formTraj(x_from, y_from, z_from, angle_from, x_to, y_to, z_to, angle_to)
    resolution = 50;
    xTraj = linspace(x_from, x_to, resolution);
    yTraj = linspace(y_from, y_to, resolution);
    zTraj = linspace(z_from, z_to, resolution);
    angleTraj = linspace(angle_from, angle_to, resolution);
    % xTraj = zeros(1, resolution);
    % yTraj = zeros(1, resolution);
    % zTraj = zeros(1, resolution);
    % angleTraj = zeros(1, resolution);
    % 
    % j = 0;
    % for i = (1 :resolution)
    %         xTraj(i) = x_from + 3/resolution^2*(x_from-x_to)*j^2 - 2/resolution^3*(x_from-x_to)*j^3;
    %         yTraj(i) = y_from + 3/resolution^2*(y_from-y_to)*j^2 - 2/resolution^3*(y_from-y_to)*j^3;
    %         zTraj(i) = z_from + 3/resolution^2*(z_from-z_to)*j^2 - 2/resolution^3*(z_from-z_to)*j^3;
    %         angleTraj(i) = angle_from + 3/resolution^2*(angle_from-angle_to)*j^2 - 2/resolution^3*(angle_from-angle_to)*j^3;
    %     j = j + 1;
    % end
end

function move_to(position, rotating, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION) %position is (x,y,stacklvl)
    % If position is out of range, enforce horizontal gripper
    % else enforce vertical gripper (pointing down)
    if(rotating)
        angle_desired = 0;
        z = 100;
    else
        if(check_radius(position))
            angle_desired = 0;
        else
            angle_desired = 90;
        end
        z = 130;
        disp(angle_desired);
    end
    % Assign  position
    x = position(1);
    y = position(2);
    %z = 130; % set value high enough to avoid collision
    [x_from, y_from, z_from, angle_from] = currentangles(port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION);
    fprintf('x:%03d, y: %03d, z: %03d, angle: %03d\n',x_from, y_from, z_from, angle_from)
    % disp(x_from);
    % disp(y_from);
    % disp(z_from);
    % disp(angle_from);
    [xTraj, yTraj, zTraj, angleTraj] = formTraj(x_from, y_from, z_from, angle_from, x, y, z, angle_desired);
    %assignin('base', 'xTraj', xTraj);
    % Perform IK
    [position1, position2, position3, position4] = IK(xTraj, yTraj, zTraj, angleTraj); 
    % Write to DXL1-4
    for i = 1:length(position1)
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_GOAL_POSITION, position1(i));
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_GOAL_POSITION, position2(i));
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_GOAL_POSITION, position3(i));
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_GOAL_POSITION, position4(i));
    end
end

function stack_level = pick(position, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION) % Pick up cube according to stack level
    dxl_present_position5 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_PRESENT_POSITION);
    %closed = 2500;
    %open = 1800;
    closed = 3760;
    open = 4260;
    opening = linspace(dxl_present_position5, open, 10);
    for i = 1:length(opening)
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_GOAL_POSITION, opening(i));
    end
    pause(2);
    x = position(1);
    y = position(2);
    % I am commenting this line out z = 140;
    if(position(3) == 0)
        disp("Error: cannot pick from empty cube holder");
    elseif(position(3) == 1)
        %z = 40;
        z = 52;
    elseif(position(3) == 2)
        %z = 62;
        z = 74;
    elseif(position(3) == 3)
        %z = 85;
        z = 97;
    end
    if(check_radius(position))
        angle_desired = 0;
        if(position(3) == 0)
            disp("Error: cannot pick from empty cube holder");
        elseif(position(3) == 1)
            %z = 40;
            z = 44;
        elseif(position(3) == 2)
            %z = 62;
            z = 62;
        elseif(position(3) == 3)
            %z = 85;
            z = 100;
        end
    else
        angle_desired = 90;
    end
    % Lower
    [x_from, y_from, z_from, angle_from] = currentangles(port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION);
    [xTraj, yTraj, zTraj, angleTraj] = formTraj(x_from, y_from, z_from, angle_from, x, y, z, angle_desired);
    [position1, position2, position3, position4] = IK(xTraj, yTraj, zTraj, angleTraj);      % Write to DXL1-4     
    for i = 1:length(position1)
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_GOAL_POSITION, position1(i));
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_GOAL_POSITION, position2(i));
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_GOAL_POSITION, position3(i));
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_GOAL_POSITION, position4(i));
    end
 
    % Close gripper
    pause(1);
    dxl_present_position5 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_PRESENT_POSITION);
    closing = linspace(dxl_present_position5, closed, 10);
    for i = 1:length(closing)
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_GOAL_POSITION, closing(i));
    end
    pause(1);
    % Raise
    x = position(1);
    y = position(2);
    z = 120;
    [x_from, y_from, z_from, angle_from] = currentangles(port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION);
    [xTraj, yTraj, zTraj, angleTraj] = formTraj(x_from, y_from, z_from, angle_from, x, y, z, angle_desired);
    [position1, position2, position3, position4] = IK(xTraj, yTraj, zTraj, angleTraj);      % Write to DXL1-4     
    for i = 1:length(position1)
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_GOAL_POSITION, position1(i));
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_GOAL_POSITION, position2(i));
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_GOAL_POSITION, position3(i));
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_GOAL_POSITION, position4(i));
    end
 
    stack_level = position(3) - 1;
end

function stack_level = place(position, rotating, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION) % Place cube according to stack level
    x = position(1);
    y = position(2);
    %z = 140;
    if(position(3) == 0)
        %z = 40;
        z = 48;
    elseif(position(3) == 1)
        %z = 62;
        z = 70;
    elseif(position(3) == 2)
        %z = 85;
        z = 92;
    end
    if(rotating)
        x = position(1) - 5;
        y = position(2) + 5;
        angle_desired = 0;
    else
        if(check_radius(position))
            angle_desired = 0;
        else
            angle_desired = 90;
        end
    end
    disp(angle_desired);

    % Lower
    [x_from, y_from, z_from, angle_from] = currentangles(port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION);
    [xTraj, yTraj, zTraj, angleTraj] = formTraj(x_from, y_from, z_from, angle_from, x, y, z, angle_desired);
    [position1, position2, position3, position4] = IK(xTraj, yTraj, zTraj, angleTraj);      % Write to DXL1-4     
    for i = 1:length(position1)
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_GOAL_POSITION, position1(i));
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_GOAL_POSITION, position2(i));
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_GOAL_POSITION, position3(i));
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_GOAL_POSITION, position4(i));
    end
    pause(1)
    % Open gripper and release cube
    dxl_present_position5 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_PRESENT_POSITION);
    %closed = 2500;
    %open = 1800;
    closed = 3760;
    open = 4260;
    opening = linspace(dxl_present_position5, open, 10);
    for i = 1:length(opening)
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_GOAL_POSITION, opening(i));
    end
    % Raise
    x = position(1);
    y = position(2);
    z = 140;
    [x_from, y_from, z_from, angle_from] = currentangles(port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION);
    [xTraj, yTraj, zTraj, angleTraj] = formTraj(x_from, y_from, z_from, angle_from, x, y, z, angle_desired);
    [position1, position2, position3, position4] = IK(xTraj, yTraj, zTraj, angleTraj);      % Write to DXL1-4     
    for i = 1:length(position1)
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_GOAL_POSITION, position1(i));
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_GOAL_POSITION, position2(i));
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_GOAL_POSITION, position3(i));
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_GOAL_POSITION, position4(i));
    end
 
    stack_level = position(3) + 1;
end

% empty_position = holder_pos4
function rotate(position, empty_position, no_of_rotations, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION)
    % Move to holder_pos4 before rotating
    rotate_pos4 = [115, -114, 0];
    position(3) = pick(position, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
    move_to(rotate_pos4, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
    move_to(empty_position, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
    empty_position(3) = place(empty_position, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
    %flip(empty_position, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION); 
    %pick(empty_position, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
    %move_to(position, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
    %place(position, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
    if(no_of_rotations == 1)
        move_to(empty_position, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        empty_position(3) = pick(empty_position, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        move_to(rotate_pos4, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        move_to(empty_position, 1, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        empty_position(3) = place(empty_position, 1, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
    elseif(no_of_rotations == 2)
        move_to(empty_position, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        empty_position(3) = pick(empty_position, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        move_to(rotate_pos4, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        move_to(empty_position, 1, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION); 
        empty_position(3) = place(empty_position, 1, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        
        move_to(empty_position, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        empty_position(3) = pick(empty_position, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        move_to(rotate_pos4, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        move_to(empty_position, 1, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        empty_position(3) = place(empty_position, 1, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
   elseif(no_of_rotations == 3)
        move_to(empty_position, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        empty_position(3) = pick(empty_position, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        move_to(rotate_pos4, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        move_to(empty_position, 1, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        empty_position(3) = place(empty_position, 1, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);

        move_to(empty_position, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        empty_position(3) = pick(empty_position, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        move_to(rotate_pos4, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        move_to(empty_position, 1, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        empty_position(3) = place(empty_position, 1, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);

        move_to(empty_position, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        empty_position(3) = pick(empty_position, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        move_to(rotate_pos4, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        move_to(empty_position, 1, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        empty_position(3) = place(empty_position, 1, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
    end
    % DO NOT Return to original position
    %pick(empty_position, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
    %move_to(position, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
    %place(position, 0, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
end

function flip(position, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION) % Rotate cube 90 degrees
        pick(position, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
        % Code to rotate block
        x = position(1);
        y = position(2);
        z = 140;
        angle_desired = 0;
        [x_from, y_from, z_from, angle_from] = currentangles(port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, ADDR_PRO_PRESENT_POSITION);
        [xTraj, yTraj, zTraj, angleTraj] = formTraj(x_from, y_from, z_from, angle_from, x, y, z, angle_desired);
        [position1, position2, position3, position4] = IK(xTraj, yTraj, zTraj, angleTraj);      % Write to DXL1-4     
        for i = 1:length(position1)
          write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_GOAL_POSITION, position1(i));
          write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_GOAL_POSITION, position2(i));
          write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_GOAL_POSITION, position3(i));
          write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_GOAL_POSITION, position4(i));
        end
        place(position, 1, port_num, PROTOCOL_VERSION, DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, ADDR_PRO_PRESENT_POSITION, ADDR_PRO_GOAL_POSITION);
end
