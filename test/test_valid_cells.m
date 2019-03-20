clear all
hold on

problem = prob.load('ship_viknes830.json', 'sjernaroyane.scenario.json');
gnc_settings = jsondecode(fileread('viknes830.gnc.json'));

xx0 = [525, 525, pi/2, 0,0,0]';
aux0 = 0;
c0 = 0;
cell_size = 50;
r = sqrt(2)*cell_size*2;

test_grid = hybrid_a_star.initialize_grid(problem.scenario, cell_size);


%%
%settings.X_density = 2;
%settings.X_limit = [5000, 8000];
%settings.N_density = 10;
%settings.N_limit = [-1, 1]*500;
%settings.r_max = 0.35;

tic
[cells, all_cells] = hybrid_a_star.valid_cells(problem.ship, test_grid, xx0, aux0, c0, gnc_settings);
toc



%% Plot
%figure(1);
%clf
for i=1:length(all_cells)
    test_grid.plot_cell(all_cells(i).y, all_cells(i).x, 'r');
end
for i=1:length(all_cells)
    plot(all_cells(i).traj.xx(2,:), all_cells(i).traj.xx(1,:), 'g', 'linewidth', 2.0)
end

for i=1:length(cells)
    plot(cells(i).traj.xx(2,:), cells(i).traj.xx(1,:), 'b', 'linewidth', 2.0)
    plot(cells(i).xx(2), cells(i).xx(1), 'b*', 'linewidth', 2.0)
end


fimplicit(@(x,y) (x-xx0(1)).^2 + (y-xx0(2)).^2 -r^2, 'k--')
axis equal
grid on