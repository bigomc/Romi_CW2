function sensors = check_sensors(x,y,heading,MAP)
    sensors = zeros(3,1);
    switch heading 
       case 1   %UP
           if MAP(x-1,y)==-1
                sensors(1) = 1;
           end
           if MAP(x,y+1)==-1
                sensors(2) = 1;
           end
           if MAP(x+1,y)==-1
                sensors(3) = 1;
           end
       case 2   %DOWN
          if MAP(x+1,y)==-1
                sensors(1) = 1;
           end
           if MAP(x,y-1)==-1
                sensors(2) = 1;
           end
           if MAP(x-1,y)==-1
                sensors(3) = 1;
           end
       case 3   %LEFT
          if MAP(x,y-1)==-1
                sensors(1) = 1;
           end
           if MAP(x-1,y)==-1
                sensors(2) = 1;
           end
           if MAP(x,y+1)==-1
                sensors(3) = 1;
           end
       case 4   %RIGHT
          if MAP(x,y+1)==-1
                sensors(1) = 1;
           end
           if MAP(x+1,y)==-1
                sensors(2) = 1;
           end
           if MAP(x,y-1)==-1
                sensors(3) = 1;
           end
       otherwise
          sensors = zeros(3,1);
    end
end