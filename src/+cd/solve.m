function result = generate(problem)

    scenario = problem.scenario;

    % Generate grid
    grid = cd.initialize_grid(scenario);

    % Initialize search variables for Dijkstra (visited cells, estimated
    % distance and parents)
    C = zeros(size(grid.cells));
    O = zeros(size(grid.cells));
    O(grid.startCell(1),grid.startCell(2)) = 1;
    D_0 = inf(size(grid.cells));
    D_0(grid.startCell(1), grid.startCell(2)) = 0;
    D_f = inf(size(grid.cells));
    D_f(grid.startCell(1), grid.startCell(2)) = sqrt((grid.endCell(1)-grid.startCell(1))^2 + (grid.endCell(2)-grid.startCell(2))^2);
    parents = zeros(size(grid.cells,1),size(grid.cells,2),2);
    
    while(1)
        % Select the cell that is not yet visited and has lowest estimated
        % distance
        c = minimum(O,D_f);
        
        if isempty(c)
            result = cd.Result(scenario, grid, [], []);
            return
        end
        
        if reached_goal_cell(grid.endCell, c)
            grid.C = C;
            grid_p = grid_path(c, parents);
            p = path(grid, grid_p);
            result = cd.Result(scenario, grid, grid_p, p);  
            return
        end
        
        
        % Mark current cell as visited before continuing
        O(c(1),c(2)) = 0;
        C(c(1),c(2)) = 1;
        
        % Four valid directions (up, down, left, right) that all increase
        % time
        density = 3;
        directions = zeros(2,density*2);
        for dx=-density:1:density
            for dy=-density:1:density
                directions = [directions [dx,dy]'];
            end
        end
        %directions = [1,  0, -1, 0, 1, 1, -1, -1;
        %               0, -1, 0, 1, 1, -1, -1, 1];
                   
        % Go through all valid directions
        for d=1:size(directions,2)
            dX = directions(1,d);
            dY = directions(2,d);
            % If the next cell is inside
            % the bounds of the grid...
            if c(1)+dX > size(D_0,1) || c(1)+dX < 1 || ...
               c(2)+dY > size(D_0,2) || c(2)+dY < 1
                continue; 
            end
            % ... and if not yet visited and it's estimated
            % distance is larger than the new estimated distance from the
            % current cell...
            x = c(1)+dX;
            y = c(2)+dY;
            
            if C(x,y)
                continue;
            end
            
            if ~O(x,y)
                if ~scenario.is_area_free([grid.X(x) grid.X(x+1)], [grid.Y(y) grid.Y(y+1)])
                    grid.cells(x,y) = 1;
                    C(x,y) = 1;
                    continue
                end
            end
            
            d = D_0(c(1),c(2)) + sqrt(dX^2+dY^2);
            
            
            
            if ~O(x,y)
                O(x,y) = 1;
            elseif d >= D_0(x,y)
                continue;
            end
            parents(x,y,:) = c;
            D_0(x,y) = d;
            D_f(x,y) = D_0(x,y) ...
                + sqrt((grid.endCell(1)-x)^2 + (grid.endCell(2)-y)^2);
            
        end
    end
end

% Helper function to determine if the goal has been reached, and if so,
% what cell was reached
function c = reached_goal_cell(end_cell, c)
    c = end_cell(1) == c(1) && end_cell(2) == c(2);
end

% Helper function to generate grid path by traversing from a given cell
% through the parents
function p = grid_path(c, parents)
    p = [c];
    while prod(parents(c(1),c(2),:)) > 0
        c = squeeze(parents(c(1),c(2),:))';
        p = [c; p];
    end
end

% Helper function to translate a grid path for a given grid into a real
% path in the environment
function p = path(grid, grid_path)
    p = zeros(size(grid_path,1),2);
    for t=1:size(grid_path,1)
        x = (grid.X(grid_path(t,1)) + grid.X(grid_path(t,1)+1))/2;
        y = (grid.Y(grid_path(t,2)) + grid.Y(grid_path(t,2)+1))/2;
        p(t,:) = [x, y];
    end
end

% Helper function to find the cell in the grid of lowest estimated distance
% that is not yet visited
function cb = minimum(O,D)
    cb = [];
    minVal = inf;
    for n=1:size(D,1)
        for e=1:size(D,2)
            if O(n,e) && D(n,e) <= minVal
                cb = [n, e];
                minVal = D(n,e);
            end
        end
    end
end