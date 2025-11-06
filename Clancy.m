
% MazeSolver  Simple maze solver supporting BFS, DFS, and A* (Manhattan/Euclid)
% Usage:
%   ms = MazeSolver(mazeMatrix, start, goal);
%   path = ms.solve('bfs');
%   ms.plot(path);
%
% mazeMatrix: 2D numeric or logical. 0 = free, 1 = wall (or false/true).
%
% Methods:
%   solve(method) - method = 'bfs' (default), 'dfs', 'astar'
%   plot(path) - visualize maze with path


% Define constants for maze values
UNKNOWN = 2;
WALL = 1;
FREE = 0;
VISITED = 3;
CURRENT = 4;

% Define color constants
blue = 2;    % pickup zone
yellow = 4;  % start zone
green = 3;   % drop off zone

% Initialize maze and robot state
AutonomousMode = true; % whether autonomous mode is active or not
maze = [2 2 2 2 2; 
       2 2 2 2 2; 
       2 2 2 2 2; 
       2 2 2 2 2; 
       2 2 2 2 2; 
       2 2 2 2 2; 
       2 2 2 2 2; 
       2 2 2 2 2; 
       2 2 2 2 2;
       2 2 2 2 2; 
       2 2 2 2 2]; % 2 = unknown, 1 = wall, 0 = free, 3 = visited, 4 = current

% Initialize robot position (middle of maze)
robotRow = 6;    % Start in middle row
robotCol = 3;    % Start in middle column
% 0 = North, 90 = East, 180 = South, 270 = West
robotOrientation = 0;  % Start facing North

% Update initial position
maze(robotRow, robotCol) = CURRENT;
       heading = brick.gyroSensor(1);
       colorSensor = brick.ColorSensor(4);  % Adjust port number as needed
       distance = brick.UltrasonicSensor(3); % Adjust port number as needed
       



        function nextMove()
            if AutonomousMode
                getNextCommand()
            end
            if ~AutonomousMode
                userControl()
            end
        end
        function userControl()
            f = figure('Name', 'EV3 Keyboard Control', ...
           'KeyPressFcn', @keyDown, ...
           'KeyReleaseFcn', @keyUp, ...
           'CloseRequestFcn', @endProgram);

            setappdata(f, 'brick', brick);
            setappdata(f, 'running', true);

            while ishandle(f) && getappdata(f, 'running')
                pause(0.1);
                drawnow;
            end

            if ishandle(f)
                close(f);
            end
            
            brick.StopAllMotors();
            disp('Program ended.');

% ======= Functions =======
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
            switch key
                case {'w','a','s','d'}
                    brick.StopMotor('AD', 'Brake');
                case {'p','l'}
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

        function updateMaze()
            % Updates the maze based on current sensor readings and position
            global maze robotRow robotCol robotOrientation;
            
            % Mark current position as visited
            maze(robotRow, robotCol) = VISITED;
            
            % Get distance reading
            dist = readLeftDistance();
            
            % Based on robot's orientation, update appropriate cells
            switch robotOrientation
                case 0  % Facing North
                    checkWall(robotRow, robotCol-1, dist);  % Check West
                case 90 % Facing East
                    checkWall(robotRow-1, robotCol, dist);  % Check North
                case 180 % Facing South
                    checkWall(robotRow, robotCol+1, dist);  % Check East
                case 270 % Facing West
                    checkWall(robotRow+1, robotCol, dist);  % Check South
            end
            
            % Update current position
            maze(robotRow, robotCol) = CURRENT;
            
            % Display the updated maze
            displayMaze();
        end
        
        function checkWall(row, col, distance)
            % Checks if there's a wall at the given position based on sensor reading
            global maze;
            
            % Only update if the position is within bounds
            if row >= 1 && row <= size(maze,1) && col >= 1 && col <= size(maze,2)
                if distance < 0.2  % If distance is less than 20cm
                    maze(row, col) = WALL;
                else
                    maze(row, col) = FREE;
                end
            end
        end
        
        function displayMaze()
            % Displays the maze with ASCII characters
            global maze;
            
            % Define display characters
            symbols = ' #.@?';  % FREE=0, WALL=1, VISITED=3, CURRENT=4, UNKNOWN=2
            
            % Print the maze
            fprintf('\nCurrent Maze State:\n');
            for i = 1:size(maze,1)
                for j = 1:size(maze,2)
                    fprintf('%c ', symbols(maze(i,j) + 1));
                end
                fprintf('\n');
            end
            fprintf('\n');
        end

      


        %runs throught the maze with no knowledge of the maze
        function mazeNavigation()
            % Simple left-hand wall follower that looks for a green goal
            % The maze is 6x3 with 0.6m squares. This routine uses
            % - a color sensor to detect the green goal (must be available via a helper)
            % - a back touch sensor to detect collisions from behind
            % - a left distance (sonic) sensor to follow walls on the left
            %
            % Notes / assumptions:
            % - The EV3 "brick" and sensors should be attached to the object
            %   (or accessible in the workspace). The helper read* methods
            %   below try to use properties on this object (e.g. obj.distance)
            %   if present. Adapt those helpers to match your EV3 API if needed.

            % Parameters (tweak these for your robot)
            squareSize = 0.6;         % meters per cell
            forwardSpeedPower = 40;  % motor power used for forward movement (1..100)
            forwardSpeedMps = 0.20;  % approximate forward speed in m/s (used to compute time)
            leftCloserTol = 0.07;    % m, desired distance to left wall tolerance
            desiredLeftDistance = 0.12; % m target distance from left wall
            samplePause = 0.05;      % loop sleep time

            % safety: maximum steps to avoid infinite loops during debugging
            maxSteps = 1000;
            steps = 0;

            found = false;

            while ~found && steps < maxSteps
                steps = steps + 1;

                % check color sensor first (high priority)
                c = readColor();
                if strcmpi(c, 'yellow') || strcmpi(c, 'y') || c==3
                    % green detected — stop and announce
                    stopMotors();
                    disp('Goal (green) detected — stopping.');
                    found = true;
                    break;
                end

                % (Back touch sensor removed) — no back-touch collision recovery

                % left-hand wall-following: read left distance
                leftDist = readLeftDistance();

                if isempty(leftDist) || isnan(leftDist)
                    % if left distance not available, just move forward one cell
                    moveForward(squareSize, forwardSpeedPower, forwardSpeedMps);
                    pause(samplePause);
                    continue;
                end

                % If there is no wall on the left, try to turn left and advance
                if leftDist > (desiredLeftDistance + leftCloserTol)
                    % open on the left -> turn left into it and move forward
                    disp('Opening on the left — turning left into corridor');
                    turnLeft(0.45);
                    moveForward(squareSize, forwardSpeedPower, forwardSpeedMps);
                    pause(samplePause);
                    continue;
                end

                % If left distance is too small, steer right a bit
                if leftDist < (desiredLeftDistance - leftCloserTol)
                    % too close to left wall — steer right slightly while moving
                    disp('Too close to left wall — steering right');
                    setMotorPower(forwardSpeedPower, round(forwardSpeedPower*0.6));
                    pause(0.25);
                    stopMotors();
                    continue;
                end

                % Otherwise move forward one cell
                moveForward(squareSize, forwardSpeedPower, forwardSpeedMps);
                pause(samplePause);
            end

            if ~found
                disp('Finished navigation loop (max steps reached) without finding goal.');
            end

        end


        function turnToHeading(targetHeading)
            % Turn to a specific heading using the gyro sensor
            % targetHeading should be in degrees (0-359)
            global robotOrientation;
            
            currentHeading = gyroSensor(brick);
            
            % Calculate the shortest turning direction
            diff = mod(targetHeading - currentHeading + 180, 360) - 180;
            
            % Set turning power based on the magnitude of the turn
            p = min(35, max(20, abs(diff) / 2));
            
            while abs(diff) > 2  % 2-degree tolerance
                currentHeading = gyroSensor(brick);
                diff = mod(targetHeading - currentHeading + 180, 360) - 180;
                
                if diff > 0
                    % Turn left
                    setMotorPower(-p, p);
                else
                    % Turn right
                    setMotorPower(p, -p);
                end
                pause(0.05);
            end
            
            stopMotors();
            pause(0.05);
            
            % Update robot's orientation
            robotOrientation = mod(targetHeading, 360);
            
            % Update the maze display after turning
            updateMaze();
        end

        function turnLeft()
            % Turn left 90 degrees using gyro sensor
            currentHeading = gyroSensor(brick);
            targetHeading = mod(currentHeading - 90, 360);  % Subtract 90 degrees
            turnToHeading(targetHeading);
        end

        function turnRight()
            % Turn right 90 degrees using gyro sensor
            currentHeading = gyroSensor(brick);
            targetHeading = mod(currentHeading + 90, 360);  % Add 90 degrees
            turnToHeading(targetHeading);
        end

        function moveForward(distance, power, speedMps)
            % Move forward approx `distance` meters while maintaining heading
            global robotRow robotCol robotOrientation;
            
            if nargin < 4 || isempty(speedMps)
                speedMps = 0.20;
            end
            if nargin < 3 || isempty(power)
                power = 40;
            end
            if nargin < 2 || isempty(distance)
                distance = 0.6;
            end
            
            targetHeading = gyroSensor(brick);  % Get initial heading
            t = max(0.05, distance / speedMps);
            startTime = tic;
            
            while toc(startTime) < t
                currentHeading = gyroSensor(brick);
                headingError = mod(targetHeading - currentHeading + 180, 360) - 180;
                
                % Adjust motor powers based on heading error
                correction = min(10, abs(headingError)) * sign(headingError);
                leftPower = power - correction;
                rightPower = power + correction;
                
                setMotorPower(leftPower, rightPower);
                pause(0.05);
            end
            
            stopMotors();
            pause(0.05);
            
            % Update robot position based on orientation
            switch robotOrientation
                case 0      % Facing North
                    robotRow = robotRow - 1;
                case 90     % Facing East
                    robotCol = robotCol + 1;
                case 180    % Facing South
                    robotRow = robotRow + 1;
                case 270    % Facing West
                    robotCol = robotCol - 1;
            end
            
            % Update the maze display
            updateMaze();
        end

        function moveBackward(distance, power)
            % Move backward approx `distance` meters (time-estimated).
            if nargin < 3 || isempty(power)
                power = 30;
            end
            if nargin < 2 || isempty(distance)
                distance = 0.12;
            end
            speedMps = 0.15;
            t = max(0.05, distance / speedMps);
            setMotorPower(-power, -power);
            pause(t);
            stopMotors();
            pause(0.05);
        end

        function navigate()
            % alias
            mazeNavigation();
        end

        % --- sensor/motor helper methods ---
        function c = readColor()
            % Return detected color name or numeric code. 'unknown' if unavailable.
            c = 'unknown';
            try
                if ~isempty(color)
                    c = readColor(color);
                elseif isprop('colorSensor') && ~isempty(colorSensor)
                    c = readColor(colorSensor);
                end
            catch
                % leave as 'unknown'
            end
        end

        function d = readLeftDistance()
            % Return left ultrasonic distance in meters, NaN if not available
            d = NaN;
            try
                if ~isempty(distance)
                    if exist('readDistance', 'file')==2
                        d = readDistance(distance);
                    elseif ismethod(distance, 'read')
                        d = distance.read();
                    else 
                        d = distance.Distance;
                    end
                end
            catch
                d = NaN;
            end
        end

        % back touch sensor removed — no readBackTouch() helper

        function setMotorPower( leftPwr, rightPwr)
            % Set left/right motor power (A and D assumed).
            try
                if ~isempty(brick)
                    brick.MoveMotor('A', leftPwr);
                    brick.MoveMotor('D', rightPwr);
                else
                    % try global brick if present
                    if exist('brick', 'var')==1 && ~isempty(brick)
                        brick.MoveMotor('A', leftPwr);
                        brick.MoveMotor('D', rightPwr);
                    end
                end
            catch
            end
        end

        function stopMotors()
            try
                if ~isempty(brick)
                    brick.StopAllMotors('Brake');
                elseif exist('brick', 'var')==1 && ~isempty(brick)
                    brick.StopAllMotors('Brake');
                end
            catch
            end
        end

        
    

