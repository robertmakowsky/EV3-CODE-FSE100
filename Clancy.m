
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


blue; % pickup zone
yellow; % start zone
green; % drop off zone
AutonomousMode = true; % whether autonomous mode is active or not
maze  = [2 2 2 2 2; 
        2 2 2 2 2; 
        2 2 2 2 2; 
        2 2 2 2 2; 
        2 2 2 2 2; 
        2 2 2 2 2; 
        2 2 2 2 2; 
        2 2 2 2 2; 
        2 2 2 2 2;
        2 2 2 2 2; 
        2 2 2 2 2] % logical matrix: 1 = wall, 0 = free, 2 = unknown
        angle = 0    % heading angle in radians
       distance = sonicSensor(brick); % distance to nearest object in m
       heading = gyroSensor(brick);
       



        function nextMove(obj)
            if obj.AutonomousMode
                getNextCommand()
            end
            if ~obj.AutonomousMode
                userControl()
            end
        end
        function userControl(obj)
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
                    obj.AutonomousMode = true;
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

        function constrainMaze(obj)
            obj.maze
        end

        %TODO: define threshold
        function angle  = calculateHeading(obj)
            angle = atan(NextRowWallDistance  / NextRowWallDistance);
            if (angle > threshold)
                turnLeft(obj);
            end
            if (angle < threshold)
                turnRight(obj);
            end
        end


        %runs throught the maze with no knowledge of the maze
        function mazeNavigation(obj)
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
                c = obj.readColor();
                if strcmpi(c, 'green') || strcmpi(c, 'g') || c==3
                    % green detected — stop and announce
                    obj.stopMotors();
                    disp('Goal (green) detected — stopping.');
                    found = true;
                    break;
                end

                % (Back touch sensor removed) — no back-touch collision recovery

                % left-hand wall-following: read left distance
                leftDist = obj.readLeftDistance();

                if isempty(leftDist) || isnan(leftDist)
                    % if left distance not available, just move forward one cell
                    obj.moveForward(squareSize, forwardSpeedPower, forwardSpeedMps);
                    pause(samplePause);
                    continue;
                end

                % If there is no wall on the left, try to turn left and advance
                if leftDist > (desiredLeftDistance + leftCloserTol)
                    % open on the left -> turn left into it and move forward
                    disp('Opening on the left — turning left into corridor');
                    obj.turnLeft(0.45);
                    obj.moveForward(squareSize, forwardSpeedPower, forwardSpeedMps);
                    pause(samplePause);
                    continue;
                end

                % If left distance is too small, steer right a bit
                if leftDist < (desiredLeftDistance - leftCloserTol)
                    % too close to left wall — steer right slightly while moving
                    disp('Too close to left wall — steering right');
                    obj.setMotorPower(forwardSpeedPower, round(forwardSpeedPower*0.6));
                    pause(0.25);
                    obj.stopMotors();
                    continue;
                end

                % Otherwise move forward one cell
                obj.moveForward(squareSize, forwardSpeedPower, forwardSpeedMps);
                pause(samplePause);
            end

            if ~found
                disp('Finished navigation loop (max steps reached) without finding goal.');
            end

        end


        function turnLeft(obj, duration)
            % In-place left turn. duration optional (seconds).
            if nargin < 2 || isempty(duration)
                duration = 0.45;
            end
            p = 35;
            obj.setMotorPower(-p, p);
            pause(duration);
            obj.stopMotors();
            pause(0.05);
        end

        function turnRight(obj, duration)
            % In-place right turn. duration optional (seconds).
            if nargin < 2 || isempty(duration)
                duration = 0.45;
            end
            p = 35;
            obj.setMotorPower(p, -p);
            pause(duration);
            obj.stopMotors();
            pause(0.05);
        end

        function moveForward(obj, distance, power, speedMps)
            % Move forward approx `distance` meters (time-estimated).
            if nargin < 4 || isempty(speedMps)
                speedMps = 0.20;
            end
            if nargin < 3 || isempty(power)
                power = 40;
            end
            if nargin < 2 || isempty(distance)
                distance = 0.6;
            end
            t = max(0.05, distance / speedMps);
            obj.setMotorPower(power, power);
            pause(t);
            obj.stopMotors();
            pause(0.05);
        end

        function moveBackward(obj, distance, power)
            % Move backward approx `distance` meters (time-estimated).
            if nargin < 3 || isempty(power)
                power = 30;
            end
            if nargin < 2 || isempty(distance)
                distance = 0.12;
            end
            speedMps = 0.15;
            t = max(0.05, distance / speedMps);
            obj.setMotorPower(-power, -power);
            pause(t);
            obj.stopMotors();
            pause(0.05);
        end

        function navigate(obj)
            % alias
            obj.mazeNavigation();
        end

        % --- sensor/motor helper methods ---
        function c = readColor(obj)
            % Return detected color name or numeric code. 'unknown' if unavailable.
            c = 'unknown';
            try
                if isprop(obj, 'color') && ~isempty(obj.color)
                    c = readColor(obj.color);
                elseif isprop(obj, 'colorSensor') && ~isempty(obj.colorSensor)
                    c = readColor(obj.colorSensor);
                end
            catch
                % leave as 'unknown'
            end
        end

        function d = readLeftDistance(obj)
            % Return left ultrasonic distance in meters, NaN if not available
            d = NaN;
            try
                if isprop(obj, 'distance') && ~isempty(obj.distance)
                    if exist('readDistance', 'file')==2
                        d = readDistance(obj.distance);
                    elseif ismethod(obj.distance, 'read')
                        d = obj.distance.read();
                    elseif isprop(obj.distance, 'Distance')
                        d = obj.distance.Distance;
                    end
                end
            catch
                d = NaN;
            end
        end

        % back touch sensor removed — no readBackTouch() helper

        function setMotorPower(obj, leftPwr, rightPwr)
            % Set left/right motor power (A and D assumed).
            try
                if isprop(obj, 'brick') && ~isempty(obj.brick)
                    obj.brick.MoveMotor('A', leftPwr);
                    obj.brick.MoveMotor('D', rightPwr);
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

        function stopMotors(obj)
            try
                if isprop(obj, 'brick') && ~isempty(obj.brick)
                    obj.brick.StopAllMotors('Brake');
                elseif exist('brick', 'var')==1 && ~isempty(brick)
                    brick.StopAllMotors('Brake');
                end
            catch
            end
        end

        
    

