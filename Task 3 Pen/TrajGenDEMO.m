% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
%Picking up pen and Drawing lines
% % % % % % % % % % % % % % % % % % 
x12_desired = [109,100, 100,  200, 150, 150];
y12_desired = [0,   175,175,  175, 125, 175];
z12_desired = [160, 160,76,   76,  76, 76];
k_max = length(x12_desired);

resolution = 27;

%Circle
% % % % % % % % % % % % % % % % % % 
x3_desired = [x1, x2];
y3_desired = [y1, y2];
z3_desired = [77, 79];

% centre
centre = [centerX,centerY];
radius = r;

% multiple of 60
semiResolution = 60;


% Return pen
% % % % % % % % % % % % % % % % % % 
x4_desired = [175, 175, 105, 85];
y4_desired = [200, 200,   0,   0];
z4_desired = [77,  160,  160, 100];

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 


x12 = zeros(1, resolution*(k_max-1));
y12 = zeros(1, resolution*(k_max-1));
z12 = zeros(1, resolution*(k_max-1));

j = 1;
for k = 1:(k_max - 1)
    for i = (1 + (k-1)*resolution):((k)*resolution)
            x12(i) = x12_desired(k) + 3/resolution^2*(x12_desired(k+1)-x12_desired(k))*j^2 - 2/resolution^3*(x12_desired(k+1)-x12_desired(k))*j^3;
            y12(i) = y12_desired(k) + 3/resolution^2*(y12_desired(k+1)-y12_desired(k))*j^2 - 2/resolution^3*(y12_desired(k+1)-y12_desired(k))*j^3;
            z12(i) = z12_desired(k) + 3/resolution^2*(z12_desired(k+1)-z12_desired(k))*j^2 - 2/resolution^3*(z12_desired(k+1)-z12_desired(k))*j^3;
        j = j + 1;
    end
    j = 1;
end



% plot3(x12,y12,z12);

%semicircle


x3 = zeros(1, semiResolution);
y3 = zeros(1, semiResolution);
z3 = zeros(1, semiResolution);

viaSize = length(pass);

if(viaSize == 1)
    pt = pass(1);
    point1Y = Y(pt,L_Y,R_Y,T_Y,B_Y);

    for i = (1):(semiResolution/2)
        y3(i) = y3_desired(1) + 3/(semiResolution/2)^2*(point1Y-y3_desired(1))*i^2 - 2/(semiResolution/2)^3*(point1Y-y3_desired(1))*i^3;
        x3(i) = centre(1) + PN(pt, T_X, B_X, x3_desired(1)) * sqrt(radius^2 - (y3(i)-centre(2))^2);
        z3(i) = z3_desired(1);
    end
    j = 1;
    for i = (semiResolution/2+1):(semiResolution)
        y3(i) = point1Y + 3/(semiResolution/2)^2*(y3_desired(2)-point1Y)*j^2 - 2/(semiResolution/2)^3*(y3_desired(2)-point1Y)*j^3;
        x3(i) = centre(1) + PN(pt, T_X, B_X, x3_desired(2)) * sqrt(radius^2 - (y3(i)-centre(2))^2);
        z3(i) = z3_desired(1);
        j = j+1;
    end
    disp(1);
elseif(viaSize == 2)
        pt1 = pass(1);
        pt2 = pass(2);
        point1Y = Y(pt1,L_Y,R_Y,T_Y,B_Y);
        point1X = Y(pt1,L_X,R_X,T_X,B_X);
        point2Y = Y(pt2,L_Y,R_Y,T_Y,B_Y);
        point2X = Y(pt2,L_X,R_X,T_X,B_X);

        for i = (1):(semiResolution/3)
            y3(i) = y3_desired(1) + 3/(semiResolution/3)^2*(point1Y-y3_desired(1))*i^2 - 2/(semiResolution/3)^3*(point1Y-y3_desired(1))*i^3;
            x3(i) = centre(1) + PN(pt1, T_X, B_X, x3_desired(1)) * sqrt(radius^2 - (y3(i)-centre(2))^2);
            z3(i) = z3_desired(1);
        end
        j = 1;
        for i = (semiResolution/3+1):(2*semiResolution/3)
            y3(i) = point1Y + 3/(semiResolution/3)^2*(point2Y-point1Y)*j^2 - 2/(semiResolution/3)^3*(point2Y-point1Y)*j^3;
            x3(i) = centre(1) + PNvia(pt1, pt2) * sqrt(radius^2 - (y3(i)-centre(2))^2);
            z3(i) = z3_desired(1);
            j = j + 1;
        end
        j = 1;
        for i = (2*semiResolution/3+1):(semiResolution)
            y3(i) = point2Y + 3/(semiResolution/3)^2*(y3_desired(2)-point2Y)*j^2 - 2/(semiResolution/3)^3*(y3_desired(2)-point2Y)*j^3;
            x3(i) = centre(1) + PN(pt2, T_X, B_X, x3_desired(2)) * sqrt(radius^2 - (y3(i)-centre(2))^2);
            z3(i) = z3_desired(1);
            j = j + 1;
        end
        disp(2);

elseif(viaSize == 3)
        pt1 = pass(1);
        pt2 = pass(2);
        pt3 = pass(3);
        point1Y = Y(pt1,L_Y,R_Y,T_Y,B_Y);
        point1X = Y(pt1,L_X,R_X,T_X,B_X);
        point2Y = Y(pt2,L_Y,R_Y,T_Y,B_Y);
        point2X = Y(pt2,L_X,R_X,T_X,B_X);   
        point3Y = Y(pt3,L_Y,R_Y,T_Y,B_Y);
        point3X = Y(pt3,L_X,R_X,T_X,B_X);

    for i = (1):(semiResolution/4)
        y3(i) = y3_desired(1) + 3/(semiResolution/4)^2*(point1Y-y3_desired(1))*i^2 - 2/(semiResolution/4)^3*(point1Y-y3_desired(1))*i^3;
        x3(i) = centre(1) + PN(pt1, T_X, B_X, x3_desired(1)) * sqrt(radius^2 - (y3(i)-centre(2))^2);
        z3(i) = z3_desired(1);
    end
    j = 1;
    for i = (semiResolution/4+1):(2*semiResolution/4)
        y3(i) = point1Y + 3/(semiResolution/4)^2*(point2Y-point1Y)*j^2 - 2/(semiResolution/4)^3*(point2Y-point1Y)*j^3;
        x3(i) = centre(1) + PNvia(pt1, pt2) * sqrt(radius^2 - (y3(i)-centre(2))^2);
        z3(i) = z3_desired(1);
        j = j + 1;
    end
    j = 1;
    for i = (2*semiResolution/4+1):(3*semiResolution/4)
        y3(i) = point2Y + 3/(semiResolution/4)^2*(point3Y-point2Y)*j^2 - 2/(semiResolution/4)^3*(point3Y-point2Y)*j^3;
        x3(i) = centre(1) + PNvia(pt2, pt3) * sqrt(radius^2 - (y3(i)-centre(2))^2);
        z3(i) = z3_desired(1);
        j = j + 1;
    end
    j = 1;
    for i = (3*semiResolution/4+1):(semiResolution)
        y3(i) = point3Y + 3/(semiResolution/4)^2*(y3_desired(2)-point3Y)*j^2 - 2/(semiResolution/4)^3*(y3_desired(2)-point3Y)*j^3;
        x3(i) = centre(1) + PN(pt3, T_X, B_X, x3_desired(2)) * sqrt(radius^2 - (y3(i)-centre(2))^2);
        z3(i) = z3_desired(1);
        j = j + 1;
    end
    disp(3);

elseif(viaSize == 4)
        pt1 = pass(1);
        pt2 = pass(2);
        pt3 = pass(3);
        pt4 = pass(4);
        point1Y = Y(pt1,L_Y,R_Y,T_Y,B_Y);
        point2Y = Y(pt2,L_Y,R_Y,T_Y,B_Y);
        point3Y = Y(pt3,L_Y,R_Y,T_Y,B_Y);
        point4Y = Y(pt4,L_Y,R_Y,T_Y,B_Y);
    for i = (1):(semiResolution/5)
        y3(i) = y3_desired(1) + 3/(semiResolution/5)^2*(point1Y-y3_desired(1))*i^2 - 2/(semiResolution/5)^3*(point1Y-y3_desired(1))*i^3;
        x3(i) = centre(1) + PN(pt1, T_X, B_X, x3_desired(1)) * sqrt(radius^2 - (y3(i)-centre(2))^2);
        z3(i) = z3_desired(1);
    end
    j = 1;
    for i = (semiResolution/5+1):(2*semiResolution/5)
        y3(i) = point1Y + 3/(semiResolution/5)^2*(point2Y-point1Y)*j^2 - 2/(semiResolution/5)^3*(point2Y-point1Y)*j^3;
        x3(i) = centre(1) + PNvia(pt1, pt2) * sqrt(radius^2 - (y3(i)-centre(2))^2);
        z3(i) = z3_desired(1);
        j = j + 1;
    end
    j = 1;
    for i = (2*semiResolution/5+1):(3*semiResolution/5)
        y3(i) = point2Y + 3/(semiResolution/5)^2*(point3Y-point2Y)*j^2 - 2/(semiResolution/5)^3*(point3Y-point2Y)*j^3;
        x3(i) = centre(1) + PNvia(pt2, pt3) * sqrt(radius^2 - (y3(i)-centre(2))^2);
        z3(i) = z3_desired(1);
        j = j + 1;
    end
    j = 1;
    for i = (3*semiResolution/5+1):(4*semiResolution)
        y3(i) = point3Y + 3/(semiResolution/5)^2*(point4Y-point3Y)*j^2 - 2/(semiResolution/5)^3*(point4Y-point3Y)*j^3;
        x3(i) = centre(1) + PNvia(pt3, pt4) * sqrt(radius^2 - (y3(i)-centre(2))^2);
        z3(i) = z3_desired(1);
        j = j + 1;
    end
    j = 1;
    for i = (4*semiResolution/5+1):(semiResolution)
        y3(i) = point4Y + 3/(semiResolution/5)^2*(y3_desired(2)-point4Y)*j^2 - 2/(semiResolution/5)^3*(y3_desired(2)-point4Y)*j^3;
        x3(i) = centre(1) + PN(pt4, T_X, B_X, x3_desired(2)) * sqrt(radius^2 - (y3(i)-centre(2))^2);
        z3(i) = z3_desired(1);
        j = j + 1;
    end
    disp(4);

else
    for i = (1):(semiResolution)
        y3(i) = y3_desired(1) + 3/semiResolution^2*(y3_desired(2)-y3_desired(1))*i^2 - 2/semiResolution^3*(y3_desired(2)-y3_desired(1))*i^3;
        x3(i) = centre(1) - sqrt(radius^2 - (y3(i)-centre(2))^2);
        z3(i) = z3_desired(1);
    end
    disp(0);
end


x3 = [x1,x3];
y3 = [y1,y3];
z3 = [78,z3];

% plot([x3],[y3]);


if(dir == 1)
    x3 = flip(x3);
    y3 = flip(y3);
end



x4 = zeros(1, resolution*(3));
y4 = zeros(1, resolution*(3));
z4 = zeros(1, resolution*(3));


j = 1;
for k = 1:3
    for i = (1 + (k-1)*resolution):((k)*resolution)
            x4(i) = x4_desired(k) + 3/resolution^2*(x4_desired(k+1)-x4_desired(k))*j^2 - 2/resolution^3*(x4_desired(k+1)-x4_desired(k))*j^3;
            y4(i) = y4_desired(k) + 3/resolution^2*(y4_desired(k+1)-y4_desired(k))*j^2 - 2/resolution^3*(y4_desired(k+1)-y4_desired(k))*j^3;
            z4(i) = z4_desired(k) + 3/resolution^2*(z4_desired(k+1)-z4_desired(k))*j^2 - 2/resolution^3*(z4_desired(k+1)-z4_desired(k))*j^3;
            j = j + 1;
    end
    j = 1;
end


% Stitch

xTraj = [x12 x3 x4];
yTraj = [y12 y3 y4];
zTraj = [z12 z3 z4];

 plot3(xTraj,yTraj,zTraj);


function getY = Y(id, ry, ly, ty,by)
    if(id == 1)
        getY = ry;
    elseif(id == 2)
        getY = ly;
    elseif(id == 3)
        getY = ty;
    elseif(id == 4)
        getY = by;
    else
        getY = 0;
    end
end

function comparePN = PN(id, tx, bx, px)
    if(id == 1)
        comparePN = 1;
    elseif(id == 2)
            comparePN = -1;
    elseif(id == 3)
        if(tx > px)
            comparePN = -1;
        else
            comparePN = 1;
        end
    elseif(id == 4)
        if(bx > px)
            comparePN = -1;
        else
            comparePN = 1;
        end
    else
        disp('error! via point id incorrect');
    end
end


function getPNvia = PNvia(id, id2)
    if(id == 1)
        if(id2 == 4)
            getPNvia = 1;
        end
    elseif(id == 2)
        if(id2 == 3)
            getPNvia = -1;
        end
    elseif(id == 3)
        if(id2 == 1)
            getPNvia = 1;
        end
    elseif(id == 4)
        if(id2 == 2)
            getPNvia = -1;
        end
    end
end




function genSemi = genSemi(A, B, semiResolution, i_start, i_stop, centre, radius)
    j = 1;
    for i = (i_start):(i_stop)
        y3(i) = A + 3/(semiResolution)^2*(B-A)*j^2 - 2/(semiResolution)^3*(B-A)*j^3;
        x3(i) = centre(1) - sqrt(radius^2 - (y3(i)-centre(2))^2);
        j = j+1;
    end
    genSemi = [x3;y3];
end