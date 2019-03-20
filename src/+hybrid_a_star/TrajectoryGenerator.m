classdef TrajectoryGenerator
    properties
        buffer
        ship
        settings
        max_radius
    end
    methods
        
        function this = TrajectoryGenerator(ship, settings)
            this.buffer = containers.Map;
            this.ship = ship;
            this.settings = settings;
            this.max_radius = sqrt(2)*settings.cell_size*2;
        end
        
        function trajectory = generate(this, t0, xx0, aux0, c0, p_k1)
            u_d = 5;

            [t0_n, xx0_n, c0_n, p_k1_n] = this.normalize(t0, xx0, c0, p_k1);
            
            bucket = this.bucket(xx0_n, p_k1_n);
            if (this.buffer.isKey(bucket))
                trajectory_n = this.buffer(bucket);
            else
                trajectory_n = this.ship.simulate(t0_n, xx0_n, aux0, c0_n, @(xx,aux) this.controller(xx, aux, xx0_n(1:2), p_k1_n, u_d, this.ship, this.settings.gnc), @(t,xx,uu) this.cost(t,xx,uu), 100, @(xx,aux) (xx(1)-xx0_n(1))^2 + (xx(2)-xx0_n(2))^2 - this.max_radius^2 > 0);
                this.buffer(bucket) = trajectory_n;
            end
            trajectory = this.denormalize(trajectory_n, t0, xx0, c0);
        end
        
        function b = bucket(this, xx_n, p_k1_n)
            du = 1;
            dv = 1;
            dr = deg2rad(5);
            dpx = 25;
            dpy = 25;
            b = [ 'u' int2str(floor(xx_n(4)/du)) ...
                  'v' int2str(floor(xx_n(5)/dv)) ...
                  'r' int2str(floor(xx_n(6)/dr)) ...
                  'px' int2str(floor(p_k1_n(1)/dpx)) ...
                  'py' int2str(floor(p_k1_n(2)/dpy)) ];
        end
    end

        
    methods(Static)
       
        function [t_n, xx_n, c_n, p_k1_n] = normalize(t, xx, c, p_k1)
            p = xx(1:2);
            psi = xx(3);
            R = [cos(psi) -sin(psi); sin(psi) cos(psi)];  

            t_n = 0;
            xx_n = zeros(6,1);
            xx_n(4:end) = xx(4:end);
            c_n = 0;

            p_k1_n = p_k1 - p;
            p_k1_n = R'*p_k1_n;
        end

        function trajectory = denormalize(trajectory_n, t, xx, c)
            p = xx(1:2);
            psi = xx(3);
            R = [cos(psi) -sin(psi); sin(psi) cos(psi)];  


            trajectory = trajectory_n;
            trajectory.t = trajectory.t + t;
            for t=1:length(trajectory.t)
                trajectory.xx(1:2,t) = R * trajectory.xx(1:2,t);
                trajectory.xx(1:2,t) = trajectory.xx(1:2,t) + p;
            end
            trajectory.xx(3,:) = trajectory.xx(3,:) + psi;
            trajectory.c = trajectory.c + c;
            %trajectory_n.xx = trajectory.xx - 
            %xx_n = [0;0;];
            %R = [cos(theta) -sin(theta); sin(theta) cos(theta)];

        end

        function [uu,aux_dot] = controller(xx, aux, p_k, p_k1, u_d, ship, settings)
            psi_d = prob.gnc.guidance.heading(xx(1:2,:), xx(4,:), xx(5,:), p_k, p_k1, settings.guidance);
            [uu,aux_dot] = prob.gnc.control.surge_heading(xx(3,:), xx(4,:), xx(6,:), psi_d, u_d, aux, ship, settings.control);
        end

        function [c_dot] = cost(t,xx,uu)
            c_dot = norm(xx(4:6));
        end
    end
end




