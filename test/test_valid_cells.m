clear all

problem = prob.load('ship_viknes830.json', 'sjernaroyane.scenario.json');

xx0 = [510, 510, 0, 10,0,0]';
cell_size = 150;
r = sqrt(2)*cell_size/2*1.5;

test_grid = hybrid_a_star.initialize_grid(problem.scenario, cell_size);


%%
settings.X_density = 2;
settings.X_limit = [5000, 8000];
settings.N_density = 10;
settings.N_limit = [-1, 1]*500;
settings.r_max = 0.35;

tic
[cells, all_cells] = hybrid_a_star.valid_cells(problem.ship, test_grid, xx0, 0, settings);
toc



%% Plot
figure(1);
clf
hold on
for i=1:length(all_cells)
    test_grid.plot_cell(all_cells(i).y, all_cells(i).x, 'r');
end
for i=1:length(all_cells)
    plot(all_cells(i).traj.xx(2,:), all_cells(i).traj.xx(1,:), 'g', 'linewidth', 2.0)
end

for i=1:length(cells)
    plot(cells(i).traj.xx(2,:), cells(i).traj.xx(1,:), 'b', 'linewidth', 2.0)
end


fimplicit(@(x,y) (x-525).^2 + (y-525).^2 -r^2, 'k--')
axis equal
grid on