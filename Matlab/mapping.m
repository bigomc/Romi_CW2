function [MAP_R] = mapping(x,y,heading,sensors, MAP_R)
    MAP_R(x,y) = 3;
    switch heading 
       case 1   %LEFT
           if MAP_R(x,y-1)~=3 MAP_R(x,y-1) = sensors(1,1),end
           if MAP_R(x-1,y)~=3 MAP_R(x-1,y) = sensors(2,1),end
           if MAP_R(x,y+1)~=3 MAP_R(x,y+1) = sensors(3,1),end
       case 2   %UP
           if MAP_R(x-1,y)~=3 MAP_R(x-1,y) = sensors(1,1),end
           if MAP_R(x,y+1)~=3 MAP_R(x,y+1) = sensors(2,1),end
           if MAP_R(x+1,y)~=3 MAP_R(x+1,y) = sensors(3,1),end
       case 3   %RIGHT
           if MAP_R(x,y+1)~=3 MAP_R(x,y+1) = sensors(1,1),end
           if MAP_R(x+1,y)~=3 MAP_R(x+1,y) = sensors(2,1),end
           if MAP_R(x,y-1)~=3 MAP_R(x,y-1) = sensors(3,1),end 
       case 4   %DOWN
           if MAP_R(x+1,y)~=3 MAP_R(x+1,y) = sensors(1,1),end
           if MAP_R(x,y-1)~=3 MAP_R(x,y-1) = sensors(2,1),end
           if MAP_R(x-1,y)~=3 MAP_R(x-1,y) = sensors(3,1),end
        otherwise           
    end
end