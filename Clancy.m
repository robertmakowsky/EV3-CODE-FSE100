function MazeSolverMain()
% MazeSolverMain  Simple EV3 maze navigation & control interface
% Requires: MATLAB Support Package for LEGO MINDSTORMS EV3 Hardware


clc; clear; close all; %#ok<CLEAR0ARGS>

% Connect to EV3 brick (adjust name as needed)

% Maze constants
global UNKNOWN WALL FREE VISITED CURRENT
UNKNOWN = 2; WALL = 1; FREE = 0; VISITED = 3; CURRENT = 4;

% Robot and maze state
global maze robotRow robotCol robotOrientation AutonomousMode
AutonomousMode = true;

maze = repmat(UNKNOWN, 11, 5);   % 11x5 unknown maze
robotRow = 6;                    % start row (middle)
robotCol = 3;                    % start col (middle)
robotOrientation = 0;            % 0=North,90=East,180=South,270=West
maze(robotRow, robotCol) = CURRENT;

% Initialize sensors safely
gyro = safeInitSensor(@() gyroSensor(brick, 1));
color = safeInitSensor(@() colorSensor(brick, 4));
distance = safeInitSensor(@() ultrasonicSensor(brick, 3));

dispMaze();


while true
    if AutonomousMode
        disp('Autonomous mode active.');
        mazeNavigation(brick, gyro, color, distance);
        break; % stop after maze run
    else
        disp(' Manual control mode.');
        userControl(brick);
    end
end




    function s = safeInitSensor(initFcn)
        try
            s = initFcn();
        catch
            s = [];
        end
    end

    function userControl(brick)
        f = figure('Name', 'EV3 Keyboard Control', ...
            'KeyPressFcn', @keyDown, ...
            'KeyReleaseFcn', @keyUp, ...
            'CloseRequestFcn', @endProgram);
        setappdata(f, 'brick', brick);
        setappdata(f, 'running', true);

        disp('Use W/A/S/D to move, P/L to move arm, Space to exit.');
        while ishandle(f) && getappdata(f, 'running')
            pause(0.1);
            drawnow;
        end
        if ishandle(f)
            close(f);
        end
        brick.StopAllMotors();
        disp('Program ended.');

        function keyDown(src, event)
            brick = getappdata(src, 'brick');
            key = lower(event.Key);
            switch key
                case 'w'
                    brick.MoveMotor('AD', 50);
                case 's'
                    brick.MoveMotor('AD', -50);
                case 'a'
                    brick.MoveMotor('A', -20);
                    brick.MoveMotor('D', 20);
                case 'd'
                    brick.MoveMotor('A', 20);
                    brick.MoveMotor('D', -20);
                case 'p'
                    brick.MoveMotor('C', 15);
                case 'l'
                    brick.MoveMotor('C', -15);
                case ' '
                    setappdata(src, 'running', false);
                    brick.StopAllMotors();
                    disp('Exiting manual control mode.');
                    AutonomousMode = true;
            end
        end

        function keyUp(src, event)
            brick = getappdata(src, 'brick');
            key = lower(event.Key);
            if any(strcmp(key, {'w','a','s','d'}))
                brick.StopMotor('AD', 'Brake');
            elseif any(strcmp(key, {'p','l'}))
                brick.StopMotor('C', 'Brake');
            end
        end

        function endProgram(src, ~)
            brick = getappdata(src, 'brick');
            brick.StopAllMotors('Brake');
            setappdata(src, 'running', false);
            delete(src);
            disp('Window closed. Program stopped.');
        end
    end

    function mazeNavigation(brick, gyro, color, distance)
        found = false;
        steps = 0; maxSteps = 300;
        squareSize = 0.6; forwardPower = 40; speed = 0.2;

        while ~found && steps < maxSteps
            steps = steps + 1;
            c = readColorSensor(color);
            if strcmpi(c,'green')
                stopMotors(brick);
                disp('Goal reached');
                found = true; break;
            end
            leftDist = readDistanceSensor(distance);
            if leftDist > 0.18
                turnLeft(brick, gyro);
            elseif leftDist < 0.08
                turnRight(brick, gyro);
            end
            moveForward(brick, gyro, squareSize, forwardPower, speed);
        end
        if ~found
            disp('ERROR');
        end
    end

    function moveForward(brick, gyro, dist, power, speed)
        t = dist / speed;
        startHeading = readGyroSensor(gyro);
        tic;
        while toc < t
            current = readGyroSensor(gyro);
            err = mod(startHeading - current + 180, 360) - 180;
            corr = min(10, abs(err)) * sign(err);
            brick.MoveMotor('A', power - corr);
            brick.MoveMotor('D', power + corr);
            pause(0.05);
        end
        stopMotors(brick);
        switch robotOrientation
            case 0, robotRow = robotRow - 1;
            case 90, robotCol = robotCol + 1;
            case 180, robotRow = robotRow + 1;
            case 270, robotCol = robotCol - 1;
        end
        dispMaze();
    end

    function turnLeft(brick, gyro)
        h = readGyroSensor(gyro);
        target = mod(h - 90, 360);
        turnToHeading(brick, gyro, target);
        robotOrientation = mod(robotOrientation - 90, 360);
    end

    function turnRight(brick, gyro)
        h = readGyroSensor(gyro);
        target = mod(h + 90, 360);
        turnToHeading(brick, gyro, target);
        robotOrientation = mod(robotOrientation + 90, 360);
    end

    function turnToHeading(brick, gyro, target)
        h = readGyroSensor(gyro);
        diff = mod(target - h + 180, 360) - 180;
        while abs(diff) > 3
            if diff > 0
                brick.MoveMotor('A', -30); brick.MoveMotor('D', 30);
            else
                brick.MoveMotor('A', 30); brick.MoveMotor('D', -30);
            end
            h = readGyroSensor(gyro);
            diff = mod(target - h + 180, 360) - 180;
            pause(0.05);
        end
        stopMotors(brick);
    end

    function stopMotors(brick)
        try
            brick.StopAllMotors('Brake');
        catch
        end
    end

    function h = readGyroSensor(gyro)
        try
            h = mod(double(readRotationAngle(gyro)), 360);
        catch
            h = 0;
        end
    end

    function d = readDistanceSensor(distance)
        try
            d = readDistance(distance);
        catch
            d = 0.1;
        end
    end

    function c = readColorSensor(color)
        try
            c = lower(readColor(color));
        catch
            c = 'unknown';
        end
    end

    function dispMaze()
        symbols = ' #.?@'; % 0=space,1=#,2=?,3=.,4=@
        fprintf('\nMaze State:\n');
        for i = 1:size(maze,1)
            for j = 1:size(maze,2)
                fprintf('%c ', symbols(maze(i,j)+1));
            end
            fprintf('\n');
        end
        fprintf('\n');
    end

end
