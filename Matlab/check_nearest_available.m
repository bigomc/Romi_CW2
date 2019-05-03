function [x,y] = check_nearest_available(x,y, MAP_R)
[m,n] = size(MAP_R);
dist = 50;
low_dist = 50; %diagonal of all the map
x_l=0;
y_l=0;

for i=1:1:m
    for j=1:1:n
        if MAP_R(i,j) == 1
            dist = sqrt((x-i)^2+(y-j)^2);
        end
        if dist < low_dist
            low_dist = dist;
            x_l = i;
            y_l = j;
        end
    end
end

x = x_l;
y = y_l;