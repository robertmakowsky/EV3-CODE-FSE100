
brick.SetColorMode(1,2);
color = brick.ColorCode(1);
running = true;

while running
    color = brick.ColorCode(1);
    switch color
        case 1  % Forward
            brick.MoveMotor('AD', 50);
        case 2  % Backward
            brick.MoveMotor('AD', -50);
        case 3  % Left turn
            brick.MoveMotor('A', -30);
            brick.MoveMotor('D', 30);
        case 4  % Right turn
            brick.MoveMotor('A', 30);
            brick.MoveMotor('D', -30);
        case 5  % Quit program
            brick.StopAllMotors();
            running = false;
            disp('Exiting...');
        case 6  % Right turn
            brick.MoveMotor('AD', 100);
        case 7  % Quit program
            brick.MoveMotor('AD', -100);
        end
    end