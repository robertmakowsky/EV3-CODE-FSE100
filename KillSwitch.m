%brick = ConnectBrick('RUMPSHAKER');

while 1
    touch = brick.TouchPressed(2);
    if touch
        display("working");
        break;
    end


    brick.beep();
   
end


