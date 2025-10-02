brick = legoev3('Bluetooth', 'COM3');

touch = brick.TouchedPressed(1);
while 1 
    if touch
        break;
    else
        brick.beep();

end