function trajectory = generate_trajectory(xx0, aux0, c0, p_k1, max_radius, ship, settings)
    u_d = 5;
    
    
    [xx0_n, p_k1_n] = normalize(xx0, p_k1);
    trajectory_n = ship.simulate(xx0_n, aux0, c0, @(xx,aux) controller(xx, aux, xx0_n(1:2), p_k1_n, u_d, ship, settings), @(t,xx,uu) cost(t,xx,uu), 100, @(xx,aux) (xx(1)-xx0_n(1))^2 + (xx(2)-xx0_n(2))^2 - max_radius^2 > 0);
    trajectory = denormalize(trajectory_n, xx0);
end

function [xx_n, p_k1_n] = normalize(xx, p_k1)
    p = xx(1:2);
    psi = xx(3);
    R = [cos(psi) -sin(psi); sin(psi) cos(psi)];  

    xx_n = zeros(6,1);
    xx_n(4:end) = xx(4:end);
    
    p_k1_n = p_k1 - p;
    p_k1_n = R'*p_k1_n;
end

function trajectory = denormalize(trajectory_n, xx)
    p = xx(1:2);
    psi = xx(3);
    R = [cos(psi) -sin(psi); sin(psi) cos(psi)];  

    
    trajectory = trajectory_n;
    for t=1:length(trajectory.t)
        trajectory.xx(1:2,t) = R * trajectory.xx(1:2,t);
        trajectory.xx(1:2,t) = trajectory.xx(1:2,t) + p;
    end
    trajectory.xx(3,:) = trajectory.xx(3,:) + psi;
    %trajectory_n.xx = trajectory.xx - 
    %xx_n = [0;0;];
    %R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    
end

function [uu,aux_dot] = controller(xx, aux, p_k, p_k1, u_d, ship, settings)
    psi_d = prob.gnc.guidance.heading(xx(1:2,:), xx(4,:), xx(5,:), p_k, p_k1, settings.guidance);
    [uu,aux_dot] = prob.gnc.control.surge_heading(xx(3,:), xx(4,:), xx(6,:), psi_d, u_d, aux, ship, settings.control);
end

function [c_dot] = cost(t,xx,uu)
    %K_u = 999;
    c_dot = norm(xx(4:6));%1 + norm(uu)*K_u; %;
end