disp('Use W/A/S/D to move. Hold key to move, release to stop. Press Q to quit.');

% Create a figure window to capture keyboard input
f = figure('Name', 'EV3 Keyboard Control', ...
           'KeyPressFcn', @keyDown, ...
           'KeyReleaseFcn', @keyUp, ...
           'CloseRequestFcn', @endProgram);

% Store the EV3 object in the figure's data
setappdata(f, 'brick', brick);
setappdata(f, 'running', true);

% Keep the program running until the window closes or Q is pressed
while ishandle(f) && getappdata(f, 'running')
    pause(0.1);  % Keeps MATLAB responsive
end

% Cleanup
if ishandle(f)
    close(f);
end
brick.StopAllMotors();
disp('Program ended.');

% ======= Functions =======

function keyDown(~, event)
    brick = getappdata(gcf, 'brick');
    key = lower(event.Key);

    switch key
        case 'w'  % Forward
            brick.MoveMotor('AD', 50);
        case 's'  % Backward
            brick.MoveMotor('AD', -50);
        case 'a'  % Left turn
            brick.MoveMotor('A', -30);
            brick.MoveMotor('D', 30);
        case 'd'  % Right turn
            brick.MoveMotor('A', 30);
            brick.MoveMotor('D', -30);
        case 'q'  % Quit program
            setappdata(gcf, 'running', false);
            brick.StopAllMotors();
            disp('Exiting...');
    end
end

function keyUp(~, ~)
    brick = getappdata(gcf, 'brick');
    brick.StopAllMotors('Brake');  % Stop when key is released
end

function endProgram(src, ~)
    brick = getappdata(src, 'brick');
    brick.StopAllMotors('Brake');
    setappdata(src, 'running', false);
    delete(src);
    disp('Window closed. Program stopped.');
end
