%INPUT X Y Z AND ANGLES -------------------------------
x_desired = [109, 109, 200, 200, 200, 125, 200];
y_desired = [  0,   0,  60,  60, 140, 140,  60];
z_desired = [90,  160,  160, 80, 80, 80, 80];
[irow,icol] = size(x_desired);
angle_desired = zeros(1, icol);

%IK CODE -----------------------------------------------------------
[theta1, theta3, theta4, theta5] = IK(x_desired, y_desired, z_desired, angle_desired);

%THIS PLOTS TO PROVE THAT THE FUNCTION WORKS---------------------------------
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
    alpha4= 0;
    alpha5 = 0;
    alpha6 = 0;
    
    d1 = 0;
    d2 = 77;
    d3 = 0;
    d4 = 0;
    d5 = 0;
    d6 = 0;
for i = 1:icol

    T1 = DH(a0, theta1(i), alpha0, d1);
    T2 = DH(a1, 0, alpha1, d2);
    T3 = DH(a2, theta3(i), alpha2, d3);
    T4 = DH(a3, theta4(i), alpha3, d4);
    T5 = DH(a4, theta5(i), alpha4, d5);
    T6 = DH(a5, 0, alpha5, d6);
    T7 = [1, 0, 0, 0;
    0, 1, 0, -30;
    0, 0, 1, 0;
    0, 0, 0, 1];
    
    T12 = T1*T2;
    T13 = T1*T2*T3;
    T14 = T1*T2*T3*T4;
    T15 = T1*T2*T3*T4*T5;
    T16 = T1*T2*T3*T4*T5*T6;
    T17 = T16*T7;
    
    plot3(T17(1,4),T17(2,4),T17(3,4), 'x');
    hold on
    plot3([T16(1,4), T17(1,4)],[T16(2,4), T17(2,4)],[T16(3,4), T17(3,4)]);
    plot3(T16(1,4),T16(2,4),T16(3,4), 'o');  
    plot3([T15(1,4), T16(1,4)],[T15(2,4), T16(2,4)],[T15(3,4), T16(3,4)]);
    plot3([T14(1,4), T15(1,4)],[T14(2,4), T15(2,4)],[T14(3,4), T15(3,4)]);
    plot3([T13(1,4), T14(1,4)],[T13(2,4), T14(2,4)],[T13(3,4), T14(3,4)]);
    plot3([T12(1,4), T13(1,4)],[T12(2,4), T13(2,4)],[T12(3,4), T13(3,4)]);
    plot3([0, T12(1,4)],[0, T12(2,4)],[0, T12(3,4)]);
    xlabel("X-axis")
    ylabel("Y-axis")
    zlabel("Z-axis")
    xlim([-350, 350])
    ylim([-350, 350])
    zlim([-350, 350])
    axis equal
    grid on;
    
end

function T = DH(a0, theta1, alpha0, d1) 

    T = [cosd(theta1), -sind(theta1), 0, a0;
  sind(theta1)*cosd(alpha0), cosd(theta1)*cosd(alpha0), -sind(alpha0), -sind(alpha0)*d1;
  sind(theta1)*sind(alpha0), cosd(theta1)*sind(alpha0), cosd(alpha0), cosd(alpha0)*d1;
  0, 0, 0, 1];
end

%%----------------------------------IK FUNCTION
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
    [irow,icol] = size(x_desired);
    
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
    
            if (THETA3_MIN < J1b) && (J1b< THETA3_MAX) && (THETA4_MIN < J2b) && (J2b< THETA4_MAX) &&(THETA5_MIN < J3b) && (J3b< THETA5_MAX) 
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
