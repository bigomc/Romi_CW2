function [MAP_R] = mapping(x,y,heading,sensors, MAP_R)
    MAP_R(x,y) = 0;
    switch heading 
       case 1   %UP
           MAP_R(x-1,y) = sensors(1,1);
           MAP_R(x,y+1) = sensors(2,1);
           MAP_R(x+1,y) = sensors(3,1);
       case 2   %DOWN
           MAP_R(x+1,y) = sensors(1,1);
           MAP_R(x,y-1) = sensors(2,1);
           MAP_R(x-1,y) = sensors(3,1);
       case 3   %LEFT
           MAP_R(x,y-1) = sensors(1,1);
           MAP_R(x-1,y) = sensors(2,1);
           MAP_R(x,y+1) = sensors(3,1);
       case 4   %RIGHT
           MAP_R(x,y+1) = sensors(1,1);
           MAP_R(x+1,y) = sensors(2,1);
           MAP_R(x,y-1) = sensors(3,1); 
       otherwise

    end
end