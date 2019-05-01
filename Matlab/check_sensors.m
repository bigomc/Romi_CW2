function sensors = check_sensors(x,y,heading,MAP)
    sensors = ones(3,1);
    switch heading 
       case 1   %LEFT
           if MAP(x,y-1)==-1
                sensors(1) = 0;
           end
           if MAP(x-1,y)==-1
                sensors(2) = 0;
           end
           if MAP(x,y+1)==-1
                sensors(3) = 0;
           end
       case 2   %UP
          if MAP(x-1,y)==-1
                sensors(1) = 0;
           end
           if MAP(x,y+1)==-1
                sensors(2) = 0;
           end
           if MAP(x+1,y)==-1
                sensors(3) = 0;
           end
       case 3   %RIGHT
           if MAP(x,y+1)==-1
                sensors(1) = 0;
           end
           if MAP(x+1,y)==-1
                sensors(2) = 0;
           end
           if MAP(x,y-1)==-1
                sensors(3) = 0;
           end
       case 4   %DOWN
          if MAP(x+1,y)==-1
                sensors(1) = 0;
           end
           if MAP(x,y-1)==-1
                sensors(2) = 0;
           end
           if MAP(x-1,y)==-1
                sensors(3) = 0;
           end
       otherwise
          sensors = ones(3,1);
    end
        
end