% [x, y, cube stack level]
% Holder positions 1, 2, 3 - grip horizontally
holder_pos1 = [ 80, -200, 0];   % r = 215
holder_pos2 = [226,    0, 0];   % r = 226
holder_pos3 = [146,  152, 0];   % r = 210
holder_pos4 = [132, -131, 0];   % r = 185
holder_pos5 = [100,    0, 0];   % r = 100
holder_pos6 = [  0,  100, 0];   % r = 100

check_radius(holder_pos3)

function [theta1, theta3, theta4, theta5] = compute_ik(x_desired, y_desired, z_desired, angle_desired)
    a0 = 0;
    a1 = 0;
    a2 = 0;
    a3 = 130;
    a4 = 124;
    a5 = 126;
    
    alpha0 = 0;
    alpha1 = 0;
    alpha2 = -90;
    alpha3 = 0;
    alpha4 = 0;
    alpha5 = 0;
    alpha6 = 0;
    
    d1 = 0;
    d2 = 77;
    d3 = 0;
    d4 = 0;
    d5 = 0;
    d6 = 0;
    
    THETA1_MIN = -180;
    THETA1_MAX =  180;
    THETA3_MIN = -193;
    THETA3_MAX =   32;
    THETA4_MIN =  -39;
    THETA4_MAX =  169;
    THETA5_MIN = -106;
    THETA5_MAX =  125;
    
    r_desired = sqrt(y_desired.^2 + x_desired.^2);
    [irow,icol] = size(x_desired);
    
    theta1 = zeros(1, icol);
    theta2 = zeros(1, icol);
    theta3 = zeros(1, icol);
    theta4 = zeros(1, icol);
    theta5 = zeros(1, icol);
    theta6 = zeros(1, icol);
    
    for i = 1:icol
        theta1(i) = atan2d(y_desired(i), x_desired(i));
    
        r3 = r_desired(i) - a5*cosd(angle_desired(i));
        z3 = z_desired(i) + a5*sind(angle_desired(i)) - d2;
        h3 = sqrt(r3^2 + z3^2);
        
        if (a3+a4) >= h3
            disp("loop")
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
    
            if (THETA3_MIN < J1b) && (J1b < THETA3_MAX) && (THETA4_MIN < J2b) && (J2b < THETA4_MAX) &&(THETA5_MIN < J3b) && (J3b < THETA5_MAX) 
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
    %     fprintf("r3: %d  z3: %d h3: %d a: %d B: %d \n",r3,z3,  h3,  a,  B)
    %     fprintf("J1a: %d  J2a: %d J3a: %d J1b: %d J2b: %d J3bL %d",J1a, J2a, J3a, J1b, J2b, J3b)
    end
end

function rad = check_radius(position) % return 1 if radius > 213, 0 if otherwise
    if(sqrt((position(1)^2) + (position(2)^2)) > 207)
        rad = 1;
    else
        rad = 0;
    end
end

function move_to(position)
    % If position is out of range, enforce horizontal gripper
    % else enforce vertical gripper (pointing down)
    if(check_radius(position))
        angle_desired = 90;
    else
        angle_desired = 0;
    end
    % Assign  position
    x = position(1);
    y = position(2);
    z = 180; % set value high enough to avoid collision
    %x1 = start(1);
    %y1 = start(2);
    %x2 = finish(1);
    %y2 = finish(2);
    % Perform IK
    [theta1, theta3, theta4, theta5] = compute_ik(x, y, z, angle_desired);
    % Write to DXL1-4

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
    [theta1, theta3, theta4, theta5] = compute_ik(x, y, z, angle_desired);
    % Write to DXL

    % Close gripper
    pause(2);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_GOAL_POSITION, 2400);
    pause(2);
    % Raise
    x = [position(1)];
    y = [position(2)];
    z = [180];
    angle_desired(end) = [];
    [theta1, theta3, theta4, theta5] = compute_ik(x, y, z, angle_desired);
    % Write to DXL

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
    [theta1, theta3, theta4, theta5] = compute_ik(x, y, z, angle_desired);
    % Write to DXL
    
    % Open gripper and release cube

    % Raise
    x = [position(1)];
    y = [position(2)];
    z = [180];
    angle_desired(end) = [];
    [theta1, theta3, theta4, theta5] = compute_ik(x, y, z, angle_desired);
    % Write to DXL

    position(3) = position(3) + 1;
end

function rotate(position, empty_position) % Rotate cube 90 degrees
    if(check_radius(position)) % If outside range, move closer before rotating
        pick(position);
        move_to(empty_position);
        place(empty_position, 0);
    else
        pick(position);
        % Code to rotate block
        x = [position(1)];
        y = [position(2)];
        z = 140;
        angle_desired = 0;
        [theta1, theta3, theta4, theta5] = compute_ik(x, y, z, angle_desired);
        % Write to DXL
        
        place(position, 1);
    end
end
