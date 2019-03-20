clear all

problem = prob.load('ship_viknes830.json', 'sjernaroyane.scenario.json');
gnc_settings = jsondecode(fileread('viknes830.gnc.json'));
%problem.scenario.start_position = [8500; 7600];

%% Calculate
settings.cell_size = 150;
settings.gnc = gnc_settings;

tic
result = hybrid_a_star.solve(problem, settings);
toc
%% Plot method results
figure(3);
clf
plot(result);
axis equal