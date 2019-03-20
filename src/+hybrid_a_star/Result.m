classdef Result
    properties
        scenario
        grid
        grid_path
        trajectory
    end
    methods
        function this = Result(scenario, grid, grid_path, trajectory)
            this.scenario = scenario;
            this.grid = grid;
            this.grid_path = grid_path;
            this.trajectory = trajectory;
        end
        
        function plot(this)
            hold on;
            
            plot(this.scenario);
            plot(this.grid);
            
            % Grid path
            firstCellPath = false;
            for i=1:size(this.grid_path,1)
                x = this.grid_path(i,1);
                y = this.grid_path(i,2);
                h = plot_cell(this.grid, y, x, 'b', 'EdgeColor','k', 'LineWidth',0.5, 'DisplayName','Grid path');
                if firstCellPath
                    h.Annotation.LegendInformation.IconDisplayStyle = 'off';
                end
                firstCellPath = true;
            end
            
            % Trajectory
            for i=1:length(this.trajectory)
                plot(this.trajectory(i).xx(2,:), this.trajectory(i).xx(1,:), 'k', 'LineWidth',2);
            end
            % Path
%             legend(); %h_tot
            %legend([h_obstacles(1); h_grids(1); h_startCell; h_endCells(1); h_tot(1)]);
            %if length(h_gridobstacles) > 0
            %    legend([h_obstacles(1); h_grids(1); h_gridobstacles(1); h_startCell; h_endCells(1); h_tot(2)]);
            %end
            hold off;
        end
        
        
    end
end
