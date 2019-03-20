problem = prob.load('ship_viknes830.json', 'sjernaroyane.scenario.json');
gnc_settings = jsondecode(fileread('viknes830.gnc.json'));

cell_size = 50;
p_k1 = [475; 475];

xx0 = [525, 525, 0, 0,0,0]';
aux0 = 0;
c0 = 0;



trajectory_generator = hybrid_a_star.TrajectoryGenerator(problem.ship, gnc_settings, cell_size);


for i=1:5
    tic
    traj = trajectory_generator.generate(xx0, aux0, c0, p_k1);
    toc
end


figure(1)
plot(traj.xx(2,:), traj.xx(1,:), 'g', 'linewidth', 2.0)

