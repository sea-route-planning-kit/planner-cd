function [cells, all_cells] = valid_cells(ship, grid, xx0, aux0, c0, settings)
    
    [x,y] = grid.cellAtPosition(xx0(1), xx0(2));
    center = grid.cellCenter(x,y);
    
    r = sqrt(2)*grid.cell_size*2;

    %X_density = settings.X_density;
    %X_limit = settings.X_limit;
    %N_density = settings.N_density;
    %N_limit = settings.N_limit;
    %r_max = settings.r_max;
    u_d = 5;
    
    cells = [];
    all_cells = [];
    best_distance = Inf;
    for x_d=[-1,0,1]
        for y_d=[-1,0,1]
            if (x_d == 0 && y_d == 0) 
                continue
            end
            if (x+x_d < 1 || x+x_d > length(grid.X)) || ...
               (y+y_d < 1 || y+y_d > length(grid.Y)) 
                continue
            end
            %X = X_limit(1) + (x-1)/(X_density-1) * (X_limit(2)-X_limit(1));
            %N = N_limit(1) + (n-1)/(N_density-1) * (N_limit(2)-N_limit(1));
            p_k = xx0(1:2);
            p_k1 = grid.cellCenter(x+x_d, y+y_d); %p_k + [x_d;y_d]*grid.cell_size;
            
            cell.x = x+x_d;
            cell.y = y+y_d;
            cell.traj = ship.simulate(xx0, aux0, c0, @(xx,aux) controller(xx, aux, p_k, p_k1, u_d, ship, settings), @(t,xx,uu) cost(t,xx,uu), 100, @(xx,aux) (xx(1)-center(1))^2 + (xx(2)-center(2))^2 - r^2 > 0);
            cell.xx = cell.traj.xx(:,end);
            cell.aux = cell.traj.aux(:,end);
            cell.c = cell.traj.c(:,end);
            
            all_cells = [all_cells cell];
            %[cell.x, cell.y, cell.xx_distance_from_center] = grid.cellAtPosition(cell.xx(1), cell.xx(2));
            
            
            [t_closest,xx_closest,~] = closest_state(cell.traj, p_k1);
            inCell = xx_closest(1) >= grid.X(x+x_d) && xx_closest(1) < grid.X(x+x_d)+grid.cell_size && ...
                     xx_closest(2) >= grid.Y(y+y_d) && xx_closest(2) < grid.Y(y+y_d)+grid.cell_size;

            if (inCell)
                % Cut down
                cell.traj.t = cell.traj.t(1:t_closest);
                cell.traj.xx = cell.traj.xx(:,1:t_closest);
                cell.traj.aux = cell.traj.aux(:,1:t_closest);
                cell.traj.c = cell.traj.c(1:t_closest);
                cell.xx = cell.traj.xx(:,end);
                cell.aux = cell.traj.aux(:,end);
                cell.c = cell.traj.c(:,end);
                
                cells = [cells cell];
            end

        end
    end
end

function [uu,aux_dot] = controller(xx, aux, p_k, p_k1, u_d, ship, settings)
    psi_d = prob.gnc.guidance.heading(xx(1:2,:), xx(4,:), xx(5,:), p_k, p_k1, settings.guidance);
    [uu,aux_dot] = prob.gnc.control.surge_heading(xx(3,:), xx(4,:), xx(6,:), psi_d, u_d, aux, ship, settings.control);
end

function [c_dot] = cost(t,xx,uu)
    %K_u = 999;
    c_dot = norm(xx(4:6));%1 + norm(uu)*K_u; %;
end

function [t_closest,xx_closest,distance_closest] = closest_state(traj, p)
    t_closest = -1;
    xx_closest = -1;
    distance_closest = Inf;
    for t=1:length(traj.t)
        xx = traj.xx(:,t);
        distance = norm(xx(1:2) - p);
        if distance < distance_closest
            t_closest = t;
            xx_closest = xx;
            distance_closest = distance;
        end
    end
end
