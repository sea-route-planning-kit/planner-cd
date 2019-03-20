function result = solve(problem, settings)

    scenario = problem.scenario;

    % Initialize
    grid = hybrid_a_star.initialize_grid(scenario, settings.cell_size);
    trajectory_generator = hybrid_a_star.TrajectoryGenerator(problem.ship, settings.gnc, settings.cell_size);


    % Initialize search variables for Dijkstra (visited cells, estimated
    % distance and parents)
    C = zeros(size(grid.cells));
    O = zeros(size(grid.cells));
    O(grid.startCell.p(1),grid.startCell.p(2)) = 1;
    D_0 = inf(size(grid.cells));
    D_0(grid.startCell.p(1),grid.startCell.p(2)) = 0;
    D_f = inf(size(grid.cells));
    D_f(grid.startCell.p(1),grid.startCell.p(2)) = sqrt((grid.endCell(1)-grid.startCell.p(1))^2 + (grid.endCell(2)-grid.startCell.p(2))^2);
    parents = zeros(size(grid.cells,1),size(grid.cells,2),2);
    %xx = empty(size(grid.cells));
    xx = cell(size(grid.cells));
    xx{grid.startCell.p(1),grid.startCell.p(2)} = grid.startCell.xx;
    aux = cell(size(grid.cells));
    aux{grid.startCell.p(1),grid.startCell.p(2)} = 0;
    traj = cell(size(grid.cells));
    
    while(1)
    %for i=1:60
        % Select the cell that is not yet visited and has lowest estimated
        % distance
        c = minimum(O,D_f);
        
        if isempty(c)
            break;
        end
        x_c = c(1);
        y_c = c(2);
        xx_c = xx{x_c,y_c};
        aux_c = aux{x_c,y_c};
        
        if reached_goal_cell(grid.endCell, c)
            grid.C = C;
            grid.O = O;
            grid.D_0 = D_0;
            grid.xx = xx;
            grid.traj = traj;
            grid_p = grid_path(c, parents);
            traj = trajectory(grid, grid_p);
            result = hybrid_a_star.Result(scenario, grid, grid_p, traj);  
            return
        end
        
        
        % Mark current cell as visited before continuing
        O(x_c,y_c) = 0;
        C(x_c,y_c) = 1;
        

        
        valid_cells = hybrid_a_star.valid_cells(trajectory_generator, grid, xx_c, aux_c, 0);
                   
        % Go through all valid cells
        for n=1:length(valid_cells)
            
            x_n = valid_cells(n).x;
            y_n = valid_cells(n).y;
            xx_n = valid_cells(n).xx;
            aux_n = valid_cells(n).aux;
            traj_n = valid_cells(n).traj;
            
            if C(x_n,y_n)
                continue;
            end
            
            if ~O(x_n,y_n)
                if ~scenario.is_area_free([grid.X(x_n) grid.X(x_n)+grid.cell_size], [grid.Y(y_n) grid.Y(y_n)+grid.cell_size])
                    grid.cells(x_n,y_n) = 1;
                    C(x_n,y_n) = 1;
                    continue
                end
            end
            
            d = D_0(x_c,y_c) + valid_cells(n).c; %sqrt((x_n-x_c)^2+(y_n-y_c)^2);
            
            
            
            if ~O(x_n,y_n)
                O(x_n,y_n) = 1;
            elseif d >= D_0(x_n,y_n)
                continue;
            end
            parents(x_n,y_n,:) = c;
            D_0(x_n,y_n) = d;
            D_f(x_n,y_n) = D_0(x_n,y_n) ...
                + sqrt((grid.endCell(1)-x_n)^2 + (grid.endCell(2)-y_n)^2) * 100;
            xx{x_n,y_n} = xx_n;
            aux{x_n,y_n} = aux_n;
            traj{x_n,y_n} = traj_n;
            
        end
    end
    grid.C = C;
    grid.O = O;
    grid.D_0 = D_0;
    grid.xx = xx;
    grid.traj = traj;
    result = hybrid_a_star.Result(scenario, grid, [], []);
end

% Helper function to determine if the goal has been reached, and if so,
% what cell was reached
function reached = reached_goal_cell(end_cell, c)
    reached = end_cell(1) == c(1) && end_cell(2) == c(2);
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
function traj = trajectory(grid, grid_path)
    traj = [];
    for i=1:size(grid_path,1)
        x = grid_path(i,1);
        y = grid_path(i,2);
        %x = (grid.X() + grid.X(grid_path(i,1)+1))/2;
        %y = (grid.Y(grid_path(i,2)) + grid.Y(grid_path(i,2)+1))/2;
        traj = [traj grid.traj{x,y}];
        
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