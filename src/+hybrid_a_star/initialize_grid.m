function grid = initialize_grid(scenario, cell_size)
    % Define cell size in all dimensions
    dX = cell_size;
    dY = cell_size;
    
    % Split up environment into cells according to cell size
    X = scenario.x_limit(1):dX:scenario.x_limit(2);
    Y = scenario.y_limit(1):dY:scenario.y_limit(2);
    cells = zeros(length(X)-1, length(Y)-1);
    
    % Define start cell from mission
    start_cell = [];
    for x=1:size(cells,1)
        for y=1:size(cells,2)
            if (X(x) <= scenario.start_position(1)) && (scenario.start_position(1) <= X(x+1)) && ...
               (Y(y) <= scenario.start_position(2)) && (scenario.start_position(2) <= Y(y+1))
                start_cell.p = [x;y];
                start_cell.xx = [scenario.start_position; zeros(4,1)];
                break;
            end
        end
    end
    
    % Define end cell from mission
    end_cell = [1,1];
    for x=1:size(cells,1)
        for y=1:size(cells,2)
            if (X(x) <= scenario.goal_position(1)) && (scenario.goal_position(1) <= X(x+1)) && ...
               (Y(y) <= scenario.goal_position(2)) && (scenario.goal_position(2) <= Y(y+1))
                end_cell = [x,y];
                break;
            end
        end
    end

    % Return grid, obstructed cells, start cell and end cell
    grid = hybrid_a_star.Grid(X, Y, cell_size, cells, start_cell, end_cell);
end