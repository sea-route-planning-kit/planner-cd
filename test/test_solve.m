clear all

problem = prob.load('ship_viknes830.json', 'sjernaroyane.scenario.json');

%% Calculate
settings.cell_size = 150;
settings.X_density = 2;
settings.X_limit = [2000, 8000];
settings.N_density = 10;
settings.N_limit = [-1, 1]*1500;
settings.r_max = 1;
tic
result = hybrid_a_star.solve(problem, settings);
toc
%% Plot method results
figure(3);
clf
plot(result);
axis equal