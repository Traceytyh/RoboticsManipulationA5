% position = [x, y, cube stack level]
% Holder positions 1, 2, 3 - grip horizontally
holder_pos1 = [ 80, -200, 0];   % r = 215
holder_pos2 = [226,    0, 0];   % r = 226
holder_pos3 = [146,  152, 0];   % r = 210
holder_pos4 = [132, -131, 0];   % r = 185
holder_pos5 = [100,    0, 0];   % r = 100
holder_pos6 = [  0,  100, 0];   % r = 100

check_radius(holder_pos3)
move_to(holder_pos3)

function [x, y, z, angle] = currentangles()
    position1 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_PRESENT_POSITION);
    position3 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_PRESENT_POSITION);
    position4 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_PRESENT_POSITION);
    position5 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_PRESENT_POSITION);
    thetafix = asin(0.024/0.13)/pi*180;
    theta1 = (position1 - 2048)/2048*180;
    theta3 = (position3 - 2048)/2048*180 - 90 + thetafix;
    theta4 = (position4 - 2048)/2048*180 + 90 - thetafix;
    theta5 = (position5 - 2048)/2048*180;
    r = 130*cosd(theta3)+124*cosd(theta3+theta4)+126*cosd(thet3+theta4+theta5);
    x = r*cosd(theta1);
    y = r*sind(theta1);
    z = 77+130*sind(theta3)+124*sind(theta3+theta4)+126*sind(theta3+theta4+theta5);
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
        end
    end
    thetafix = asin(0.024/0.13)/pi*180;
    position1 = theta1./180 .* 2048 + 2048;
    position2 = (theta3 - thetafix + 90)./180 .* 2048 + 2048;
    position3 = (theta4+ thetafix - 90)./180 .* 2048 + 2048;
    position4 = theta5./180 .* 2048 + 2048;
end

function rad = check_radius(position) % return 1 if radius > 207, 0 if otherwise
    if(sqrt((position(1)^2) + (position(2)^2)) > 207)
        rad = 1;
    else
        rad = 0;
    end
end
function [xTraj, yTraj, zTraj, angleTraj] = formTraj(x_from, y_from, z_from, angle_from, x_to, y_to, z_to, angle_to)
    resolution = 10;
    xTraj = zeros(1, resolution);
    yTraj = zeros(1, resolution);
    zTraj = zeros(1, resolution);
    angleTraj = zeros(1, resolution);
 
    j = 0;
    for i = (1 :resolution)
            xTraj(i) = x_to + 3/resolution^2*(x_to-x_from)*j^2 - 2/resolution^3*(x_to-x_from)*j^3;
            yTraj(i) = y_to + 3/resolution^2*(y_to-y_from)*j^2 - 2/resolution^3*(y_to-y_from)*j^3;
            zTraj(i) = z_to + 3/resolution^2*(z_to-z_from)*j^2 - 2/resolution^3*(z_to-z_from)*j^3;
            angleTraj(i) = angle_to + 3/resolution^2*(angle_to-angle_from)*j^2 - 2/resolution^3*(angle_to-angle_from)*j^3;
        j = j + 1;
    end
end

function move_to(position) %position is (x,y,stacklvl)
    % If position is out of range, enforce horizontal gripper
    % else enforce vertical gripper (pointing down)
    if(check_radius(position))
        angle_desired = 90;
    else
        angle_desired = 0;
    end
    % Assign  position
    x_to = position(1);
    y_to = position(2);
    z_to = 180; % set value high enough to avoid collision
    [x_from, y_from, z_from, angle_from] = currentangles();
    [xTraj, yTraj, zTraj, angleTraj] = formTraj(x_from, y_from, z_from, angle_from, x_to, y_to, z_to, angle_desired);
    % Perform IK
    [position1, position2, position3, position4] = IK(xTraj, yTraj, zTraj, angleTraj); 
    % Write to DXL1-4
    for i = 1:size(position1)
      write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID1, app.ADDR_PRO_GOAL_POSITION, position1);
      write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID2, app.ADDR_PRO_GOAL_POSITION, position2);
      write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID3, app.ADDR_PRO_GOAL_POSITION, position3);
      write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID4, app.ADDR_PRO_GOAL_POSITION, position4);
    end
end

function pick(position) % Pick up cube according to stack level
    x = [position(1), position(1)];
    y = [position(2), position(2)];
    z = [180];
    if(position(3) == 0)
        disp("Error: cannot pick from empty cube holder");
    elseif(position(3) == 1)
        z(end + 1) = 40;
    elseif(position(3) == 2)
        z(end + 1) = 62;
    elseif(position(3) == 3)
        z(end + 1) = 85;
    end
    if(check_radius(position))
        angle_desired = [90, 90];
    else
        angle_desired = [0, 0];
    end
    % Lower
    [position1, position2, position3, position4] = IK(xTraj, yTraj, zTraj, angleTraj);      % Write to DXL1-4     
    for i = 1:size(position1)       
        write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID1, app.ADDR_PRO_GOAL_POSITION, position1);       
        write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID2, app.ADDR_PRO_GOAL_POSITION, position2);       
        write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID3, app.ADDR_PRO_GOAL_POSITION, position3);       
        write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID4, app.ADDR_PRO_GOAL_POSITION, position4);     
    end
 
    % Close gripper
    pause(2);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_GOAL_POSITION, 2400);
    pause(2);
    % Raise
    x = [position(1)];
    y = [position(2)];
    z = [180];
    angle_desired(end) = [];
    [position1, position2, position3, position4] = IK(xTraj, yTraj, zTraj, angleTraj);      % Write to DXL1-4     
    for i = 1:size(position1)       
        write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID1, app.ADDR_PRO_GOAL_POSITION, position1);       
        write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID2, app.ADDR_PRO_GOAL_POSITION, position2);       
        write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID3, app.ADDR_PRO_GOAL_POSITION, position3);       
        write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID4, app.ADDR_PRO_GOAL_POSITION, position4);     
    end
 
    position(3) = position(3) - 1;
end

function place(position, rotating) % Place cube according to stack level
    x = [position(1), position(1)];
    y = [position(2), position(2)];
    z = [180];
    if(position(3) == 0)
        z(end + 1) = 40;
    elseif(position(3) == 1)
        z(end + 1) = 62;
    elseif(position(3) == 2)
        z(end + 1) = 85;
    end
    if(rotating)
        angle_desired = [0, 0];
    else
        if(check_radius(position))
            angle_desired = [90, 90];
        else
            angle_desired = [0, 0];
        end
    end

    % Lower
    [position1, position2, position3, position4] = IK(xTraj, yTraj, zTraj, angleTraj);      % Write to DXL1-4     
    for i = 1:size(position1)       
        write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID1, app.ADDR_PRO_GOAL_POSITION, position1);       
        write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID2, app.ADDR_PRO_GOAL_POSITION, position2);       
        write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID3, app.ADDR_PRO_GOAL_POSITION, position3);       
        write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID4, app.ADDR_PRO_GOAL_POSITION, position4);     
    end
    % Open gripper and release cube
 
    % Raise
    x = [position(1)];
    y = [position(2)];
    z = [180];
    angle_desired(end) = [];
    [position1, position2, position3, position4] = IK(xTraj, yTraj, zTraj, angleTraj);      % Write to DXL1-4     
    for i = 1:size(position1)       
        write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID1, app.ADDR_PRO_GOAL_POSITION, position1);       
        write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID2, app.ADDR_PRO_GOAL_POSITION, position2);       
        write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID3, app.ADDR_PRO_GOAL_POSITION, position3);       
        write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID4, app.ADDR_PRO_GOAL_POSITION, position4);     
    end
 
    position(3) = position(3) + 1;
end

function rotate(position, empty_position) % Rotate cube 90 degrees
    if(check_radius(position)) % If outside range, move closer before rotating
        pick(position);
        move_to(empty_position);
        place(empty_position, 0);
        rotate(empty_position, position);
        pick(empty_position);
        move_to(position);
        place(position, 0);
    else
        pick(position);
        % Code to rotate block
        x = [position(1)];
        y = [position(2)];
        z = 140;
        angle_desired = 0;
        [position1, position2, position3, position4] = IK(xTraj, yTraj, zTraj, angleTraj);      % Write to DXL1-4     
    for i = 1:size(position1)       
        write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID1, app.ADDR_PRO_GOAL_POSITION, position1);       
        write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID2, app.ADDR_PRO_GOAL_POSITION, position2);       
        write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID3, app.ADDR_PRO_GOAL_POSITION, position3);       
        write4ByteTxRx(app.port_num, app.PROTOCOL_VERSION, app.DXL_ID4, app.ADDR_PRO_GOAL_POSITION, position4);     
    end
        place(position, 1);
    end
end
