classdef Clancy
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


    properties
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
       
    end

    methods
        function obj = Clancy()
           
        end

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
                turnLeft(obj, angle);
            end
            if (angle < threshold)
                turnRight(obj, angle);
            end
        end

        function turnLeft(obj, angle)
            
                
            
        end

        function turnRight(obj,angle)
            
            
        end

        function moveForward(obj, distance)
            
            
        end

        function moveBackward(obj, distance)
            
            
        end

        function solve(obj, method)
           
           
        end

        
    end
end