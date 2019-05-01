function [cells] = check_suround(x,y, MAP_R)

cells = zeros(4,3);

%zero:Unavailable -> obstacle
%one:explored - avaliable
%two:not explored

x_e = x-1;
y_e = y;

cells(1,:) = [x_e,y_e,MAP_R(x_e,y_e)];

x_e = x;
y_e = y+1;

cells(2,:) = [x_e,y_e,MAP_R(x_e,y_e)];

x_e = x+1;
y_e = y;

cells(3,:) = [x_e,y_e,MAP_R(x_e,y_e)];

x_e = x;
y_e = y-1;

cells(4,:) = [x_e,y_e,MAP_R(x_e,y_e)];


end
