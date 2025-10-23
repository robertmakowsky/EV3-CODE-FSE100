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
        AutonomousMode = false
        maze         % logical matrix: true = wall, false = free
        angle = 0    % heading angle in radians
        nRows = 11
        nCols = 5
    end

    methods
        function obj = MazeSolver(mazeMatrix, start, goal)
            if nargin < 1
                error('Maze matrix required.');
            end
            obj.maze = logical(mazeMatrix(:,:) ~= 0); % true = wall
            [obj.nRows, obj.nCols] = size(obj.maze);

            if nargin < 2 || isempty(start)
                error('Start position required.');
            end
            if nargin < 3 || isempty(goal)
                error('Goal position required.');
            end

            validateattributes(start, {'numeric'}, {'vector','numel',2,'positive','integer'});
            validateattributes(goal, {'numeric'}, {'vector','numel',2,'positive','integer'});
            obj.start = start(:).';
            obj.goal = goal(:).';

            if ~obj.inBounds(obj.start) || obj.isWall(obj.start)
                error('Start is out of bounds or on a wall.');
            end
            if ~obj.inBounds(obj.goal) || obj.isWall(obj.goal)
                error('Goal is out of bounds or on a wall.');
            end
        end

        function setHeading(obj)
            obj.angle = atan(NextRowWallDistance  / NextRowWallDistance);
        end

        function turnAngle(obj)
            while obj.angle < currentHeading
                
            end
        end

        function path = solve(obj, method)
            if nargin < 2 || isempty(method)
                method = 'bfs';
            end
            method = lower(method);
            switch method
                case 'bfs'
                    path = obj.solveBFS();
                case 'dfs'
                    path = obj.solveDFS();
                case 'astar'
                    path = obj.solveAStar();
                otherwise
                    error('Unknown method: %s', method);
            end
        end

        function plot(obj, path)
            % Visualize maze, start, goal, and optional path
            figure;
            imagesc(~obj.maze); axis equal tight; colormap(gray); hold on;
            % start (green) and goal (red)
            plot(obj.start(2), obj.start(1), 'go', 'MarkerFaceColor','g','MarkerSize',8);
            plot(obj.goal(2), obj.goal(1), 'ro', 'MarkerFaceColor','r','MarkerSize',8);
            if nargin > 1 && ~isempty(path)
                % path is Nx2 [row col]
                plot(path(:,2), path(:,1), 'b-', 'LineWidth', 2);
                plot(path(:,2), path(:,1), 'bo', 'MarkerFaceColor','b', 'MarkerSize',4);
            end
            title('Maze Solver');
            hold off;
        end
    end

    methods (Access = private)
        function tf = inBounds(obj, rc)
            r = rc(1); c = rc(2);
            tf = r >= 1 && r <= obj.nRows && c >= 1 && c <= obj.nCols;
        end

        function tf = isWall(obj, rc)
            tf = obj.maze(rc(1), rc(2));
        end

        function idx = rc2ind(obj, rc)
            idx = sub2ind([obj.nRows, obj.nCols], rc(1), rc(2));
        end

        function rc = ind2rc(obj, idx)
            [r,c] = ind2sub([obj.nRows, obj.nCols], idx);
            rc = [r c];
        end

        function nb = neighbors(obj, idx)
            % return linear indices of valid neighbors (not walls)
            rc = obj.ind2rc(idx);
            r = rc(1); c = rc(2);
            if obj.allowDiagonal
                offsets = [ -1  0;
                             1  0;
                             0 -1;
                             0  1;
                            -1 -1;
                            -1  1;
                             1 -1;
                             1  1 ];
            else
                offsets = [ -1  0;
                             1  0;
                             0 -1;
                             0  1 ];
            end
            candidates = bsxfun(@plus, rc, offsets);
            % filter bounds
            valid = candidates(:,1) >= 1 & candidates(:,1) <= obj.nRows & ...
                    candidates(:,2) >= 1 & candidates(:,2) <= obj.nCols;
            candidates = candidates(valid, :);
            % filter walls
            mask = ~obj.maze(sub2ind([obj.nRows, obj.nCols], candidates(:,1), candidates(:,2)));
            candidates = candidates(mask, :);
            nb = sub2ind([obj.nRows, obj.nCols], candidates(:,1), candidates(:,2));
        end

        function path = reconstructPath(obj, cameFrom, goalIdx)
            if isempty(cameFrom) || goalIdx == 0
                path = [];
                return;
            end
            cur = goalIdx;
            rev = [];
            while cur ~= 0
                rc = obj.ind2rc(cur);
                rev(end+1, :) = rc; %#ok<AGROW>
                cur = cameFrom(cur);
                if numel(rev) > obj.nRows * obj.nCols, break; end
            end
            path = flipud(rev);
        end

        function path = solveBFS(obj)
            startIdx = obj.rc2ind(obj.start);
            goalIdx = obj.rc2ind(obj.goal);
            N = obj.nRows * obj.nCols;
            visited = false(N,1);
            cameFrom = zeros(N,1,'uint32');
            q = zeros(N,1,'uint32');
            head = 1; tail = 1;
            q(tail) = startIdx;
            visited(startIdx) = true;
            found = false;
            while head <= tail
                cur = q(head); head = head + 1;
                if cur == goalIdx
                    found = true;
                    break;
                end
                nb = obj.neighbors(cur);
                for k = 1:numel(nb)
                    nidx = nb(k);
                    if ~visited(nidx)
                        visited(nidx) = true;
                        cameFrom(nidx) = cur;
                        tail = tail + 1;
                        q(tail) = nidx;
                    end
                end
            end
            if ~found
                path = [];
            else
                path = obj.reconstructPath(cameFrom, goalIdx);
            end
        end

        function path = solveDFS(obj)
            startIdx = obj.rc2ind(obj.start);
            goalIdx = obj.rc2ind(obj.goal);
            N = obj.nRows * obj.nCols;
            visited = false(N,1);
            cameFrom = zeros(N,1,'uint32');
            stack = zeros(N,1,'uint32');
            top = 1;
            stack(top) = startIdx;
            visited(startIdx) = true;
            found = false;
            while top > 0
                cur = stack(top); top = top - 1;
                if cur == goalIdx
                    found = true;
                    break;
                end
                nb = obj.neighbors(cur);
                % push neighbors (order preserved)
                for k = 1:numel(nb)
                    nidx = nb(k);
                    if ~visited(nidx)
                        visited(nidx) = true;
                        cameFrom(nidx) = cur;
                        top = top + 1;
                        stack(top) = nidx;
                    end
                end
            end
            if ~found
                path = [];
            else
                path = obj.reconstructPath(cameFrom, goalIdx);
            end
        end

        function path = solveAStar(obj)
            startIdx = obj.rc2ind(obj.start);
            goalIdx = obj.rc2ind(obj.goal);
            N = obj.nRows * obj.nCols;
            INF = realmax;
            g = INF( N, 1 );
            f = INF( N, 1 );
            cameFrom = zeros(N,1,'uint32');

            g(startIdx) = 0;
            f(startIdx) = obj.heuristic(startIdx, goalIdx);

            openSet = true(N,1);
            openSet(:) = false;
            openSet(startIdx) = true;

            closed = false(N,1);

            while any(openSet)
                % get node in openSet with min f
                fvals = f;
                fvals(~openSet) = Inf;
                [~, cur] = min(fvals);
                if cur == goalIdx
                    path = obj.reconstructPath(cameFrom, goalIdx);
                    return;
                end
                openSet(cur) = false;
                closed(cur) = true;

                nb = obj.neighbors(cur);
                for k = 1:numel(nb)
                    nidx = nb(k);
                    if closed(nidx)
                        continue;
                    end
                    tentative_g = g(cur) + obj.moveCost(cur, nidx);
                    if ~openSet(nidx)
                        openSet(nidx) = true;
                    elseif tentative_g >= g(nidx)
                        continue;
                    end
                    cameFrom(nidx) = cur;
                    g(nidx) = tentative_g;
                    f(nidx) = g(nidx) + obj.heuristic(nidx, goalIdx);
                end
            end
            path = []; % no path found
        end

        function h = heuristic(obj, idx, goalIdx)
            % Manhattan if no diagonal allowed, Euclidean otherwise
            a = obj.ind2rc(idx);
            b = obj.ind2rc(goalIdx);
            if obj.allowDiagonal
                h = sqrt((a(1)-b(1))^2 + (a(2)-b(2))^2);
            else
                h = abs(a(1)-b(1)) + abs(a(2)-b(2));
            end
        end

        function c = moveCost(obj, idxA, idxB)
            a = obj.ind2rc(idxA);
            b = obj.ind2rc(idxB);
            dr = abs(a(1)-b(1));
            dc = abs(a(2)-b(2));
            if dr + dc == 1
                c = 1;
            else
                % diagonal
                c = sqrt(2);
            end
        end
    end
end