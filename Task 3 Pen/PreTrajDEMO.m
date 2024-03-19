% 
%                                 * 3 *
%                             *           *
%                           *               *
%                          *                 *
%                          2        c        1      
%                          *                 *
%                           *               *        
%                             *           *
%                                 * 4 *
% 

% point1
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
x1= 175;
y1 = 200;
% point2
x2 = 150;
y2 = 175;
r = 25;

% pass case
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
pass = [4 2];
% none: within range (special case: start and end at max and min)
% 1: pass max R
% 2: pass min L
% 3: pass top T
% 4: pass bottom B

% cases (clockwise)
% PASS 2 pts: RT TL LB BR
% PASS 3 pts: RTL TLB LBR BRT
% PASS 4 pts: RTLB TLBR LBRT BRTL


% clockwise/ccw
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
dir = 1;
% cw = 0
% ccw = 1

% select the correct center
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
c = 1;

syms x y
eqns = [(x-x1)^2 + (y-y1)^2 == r^2, (x-x2)^2 + (y-y2)^2 == r^2];
vars = [x y];
[solx, soly] = solve(eqns,vars);
x_cen = [double(solx)];
y_cen = [double(soly)];



if(length(x_cen)>1)
formatSpec1 = 'center1 is [%4.3f , %4.3f] \n';
fprintf(formatSpec1,[x_cen(1) y_cen(1)]);
formatSpec2 = 'center2 is [%4.3f , %4.3f] \n';
fprintf(formatSpec2,[x_cen(2) y_cen(2)]);
else
formatSpec2 = 'The center is [%4.3f , %4.3f] \n';
fprintf(formatSpec2,[x_cen y_cen]);
end


centerX = x_cen(c);
centerY = y_cen(c);

tb = (centerY>y1);


syms LR1
eqn1 = (centerX-LR1)^2 == r^2;
sol_LR = solve(eqn1,LR1);
LRX = double(sol_LR);
if(LRX(1) > LRX(2))
    R_X = LRX(1);
    L_X = LRX(2);
else
    R_X = LRX(2);
    L_X = LRX(1);
end
R_Y = centerY;
L_Y = centerY;

syms TB1
eqn1 = (centerX-TB1)^2 == r^2;
sol_TB = solve(eqn1,TB1);
TBY = double(sol_TB);
if(TBY(1) > TBY(2))
    T_Y = TBY(1);
    B_Y = TBY(2);
else
    T_Y = TBY(2);
    B_Y = TBY(1);
end
T_X = centerX;
B_X = centerX;


% disp(max_X);
% disp(min_X);
formatSpec1 = 'The Left point is [%4.3f,%4.3f] \n';
fprintf(formatSpec1,[L_X,L_Y]);
formatSpec2 = 'The Right point is [%4.3f,%4.3f] \n';
fprintf(formatSpec2,[R_X,R_Y]);
formatSpec3 = 'The Top point is [%4.3f,%4.3f] \n';
fprintf(formatSpec3,[T_X,T_Y]);
formatSpec4 = 'The Bottom point is [%4.3f,%4.3f] \n';
fprintf(formatSpec4,[B_X,B_Y]);


