function [cells, all_cells] = valid_cells(ship, grid, xx0, c0, settings)
    
    [x,y] = grid.cellAtPosition(xx0(1), xx0(2));
    center = [ (grid.X(x)+grid.X(x+1))/2;
               (grid.Y(y)+grid.Y(y+1))/2 ];
    
    r = sqrt(2)*grid.cell_size/2*1.5;

    X_density = settings.X_density;
    X_limit = settings.X_limit;
    N_density = settings.N_density;
    N_limit = settings.N_limit;
    r_max = settings.r_max;
    
    cells = [];
    all_cells = [];
    best_distance = Inf;
    for x=1:X_density
        for n=1:N_density
            X = X_limit(1) + (x-1)/(X_density-1) * (X_limit(2)-X_limit(1));
            N = N_limit(1) + (n-1)/(N_density-1) * (N_limit(2)-N_limit(1));

            cell.traj = ship.simulate(xx0, [], c0, @(xx,aux) controller(X,N), @(t,xx,uu) cost(t,xx,uu), 100, @(xx,aux) (xx(1)-center(1))^2 + (xx(2)-center(2))^2 - r^2 > 0 || abs(xx(6)) > r_max);
            cell.xx = cell.traj.xx(:,end);
            cell.c = cell.traj.c(:,end);
            [cell.x, cell.y, cell.xx_distance_from_center] = grid.cellAtPosition(cell.xx(1), cell.xx(2));
            
            if (cell.x == -1)
                continue;
            end
            all_cells = [all_cells cell];

            already_visited = false;
            for i_prev=1:length(cells)
                if (cell.x == cells(i_prev).x && cell.y == cells(i_prev).y)
                    already_visited = true;
                    %if (cell.c < cells(i_prev).c)
                    if (cell.xx_distance_from_center < cells(i_prev).xx_distance_from_center)
                        cells(i_prev) = cell;
                        break;
                    end
                end
            end
            
            if (~already_visited)
                cells = [cells cell];
            end
        end
    end
end

function [uu,aux_dot] = controller(X,N)
    uu = [X;N];
    aux_dot = [];
end

function [c_dot] = cost(t,xx,uu)
    K_u = 999;
    c_dot = 1 + norm(uu)*K_u; %norm(xx(4:6));
end
