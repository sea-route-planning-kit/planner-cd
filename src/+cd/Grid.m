classdef Grid
    properties
        X
        Y
        cells
        startCell
        endCell
        C 
    end
    methods
        function this = Grid(X, Y, cells, startCell, endCell)
            this.X = X;
            this.Y = Y;
            this.cells = cells;
            this.startCell = startCell;
            this.endCell = endCell;
        end
        
        function h = plot_cell(this, x,y, varargin)
            cell_shape = [this.X(x), this.X(x),   this.X(x+1), this.X(x+1), this.X(x);
                          this.Y(y), this.Y(y+1), this.Y(y+1), this.Y(y),   this.Y(y)];
            h = fill(cell_shape(1,:), cell_shape(2,:), varargin{:});
                            
        end
        
        function plot(this)
            % Plot grid obstacles and grid lines
            firstGridObstacle = false;
            for x=1:size(this.cells,1)
                for y=1:size(this.cells,2)
                    if this.cells(x,y)
                        h = plot_cell(this, y, x, 'r', 'EdgeColor','k', 'LineWidth',0.5, 'DisplayName','Grid cell obstacle');
                        if firstGridObstacle
                            h.Annotation.LegendInformation.IconDisplayStyle = 'off';
                        end
                        firstGridObstacle = true;                    
                    elseif this.C(x,y)
                        h = plot_cell(this, y, x, 'y', 'EdgeColor','k', 'LineWidth',0.5, 'DisplayName','Grid cell visited');
                        if firstGridObstacle
                            h.Annotation.LegendInformation.IconDisplayStyle = 'off';
                        end
                        firstGridObstacle = true;

                    end
                end
            end
            
            % Plot start cell
            x0 = this.startCell(1);
            y0 = this.startCell(2);
            
            plot_cell(this, y0, x0, 'b', 'EdgeColor','k', 'LineWidth',0.5,  'DisplayName','Grid path start');
            
            % Plot end cells
            x1 = this.endCell(1);
            y1 = this.endCell(2);
            
            plot_cell(this, y1, x1, 'y', 'EdgeColor','k', 'LineWidth',0.5,  'DisplayName','Grid path end');

        end
    end
end
