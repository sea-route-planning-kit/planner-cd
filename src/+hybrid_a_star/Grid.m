classdef Grid
    properties
        X
        Y
        cell_size
        cells
        startCell
        endCell
        D_0
        C 
        O
        xx
        traj
    end
    methods
        function this = Grid(X, Y, cell_size, cells, startCell, endCell)
            this.X = X;
            this.Y = Y;
            this.cell_size = cell_size;
            this.cells = cells;
            this.startCell = startCell;
            this.endCell = endCell;
        end
        
        function h = plot_cell(this, x,y, varargin)
            cell_shape = [this.X(x), this.X(x),   this.X(x)+this.cell_size, this.X(x)+this.cell_size, this.X(x);
                          this.Y(y), this.Y(y)+this.cell_size, this.Y(y)+this.cell_size, this.Y(y),   this.Y(y)];
            h = fill(cell_shape(1,:), cell_shape(2,:), varargin{:});
                            
        end
        
        function [gridX,gridY,distanceToCenter] = cellAtPosition(this, posX, posY)
            for x=1:size(this.cells,1)
                for y=1:size(this.cells,2)
                    if (posX >= this.X(x) && posX < this.X(x)+this.cell_size && ...
                        posY >= this.Y(y) && posY < this.Y(y)+this.cell_size)
                        gridX = x;
                        gridY = y;
                        center = this.cellCenter(x,y);
                        distanceToCenter = norm([posX;posY] - center);
                        return;
                    end
                end
            end
            gridX = -1;
            gridY = -1;
            distanceToCenter = -1;
        end
        
        function p = cellCenter(this, x, y)
            p = [this.X(x)+this.cell_size/2; this.Y(y)+this.cell_size/2];
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
                    elseif ~isempty(this.C) && this.C(x,y)
                        h = plot_cell(this, y, x, 'y', 'EdgeColor','k', 'LineWidth',0.5, 'DisplayName','Grid cell visited');
                        if firstGridObstacle
                            h.Annotation.LegendInformation.IconDisplayStyle = 'off';
                        end
                        firstGridObstacle = true;
                    elseif ~isempty(this.O) && this.O(x,y)
                        h = plot_cell(this, y, x, 'g', 'EdgeColor','k', 'LineWidth',0.5, 'DisplayName','Grid cell open set');
                        if firstGridObstacle
                            h.Annotation.LegendInformation.IconDisplayStyle = 'off';
                        end
                        firstGridObstacle = true;
                    end
                end
            end
            
            % Plot start cell
            x0 = this.startCell.p(1);
            y0 = this.startCell.p(2);
            
            plot_cell(this, y0, x0, 'b', 'EdgeColor','k', 'LineWidth',0.5,  'DisplayName','Grid path start');
            
            % Plot end cells
            x1 = this.endCell(1);
            y1 = this.endCell(2);
            
            plot_cell(this, y1, x1, 'y', 'EdgeColor','k', 'LineWidth',0.5,  'DisplayName','Grid path end');

            
            % Plot xx
            for x=1:size(this.cells,1)
                for y=1:size(this.cells,2)
                    if ~isempty(this.xx) && ~isempty(this.xx{x,y})
                        xx_c = this.xx{x,y};
                        plot(xx_c(2),xx_c(1),'or');
                    end
                    if ~isempty(this.traj) && ~isempty(this.traj{x,y})
                        traj_c = this.traj{x,y};
                        plot(traj_c.xx(2,:),traj_c.xx(1,:),'r');
                    end
                end
            end
            
            
            
        end
    end
end
