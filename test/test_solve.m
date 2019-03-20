clear all

problem = prob.load('ship_viknes830.json', 'sjernaroyane.scenario.json');
gnc_settings = jsondecode(fileread('viknes830.gnc.json'));
settings = jsondecode(fileread('viknes830.settings.json'));

%problem.scenario.start_position = [7000; 5000];

%% Calculate
settings.cell_size = 50;
settings.gnc = gnc_settings;

tic
result = hybrid_a_star.solve(problem, settings);
toc
%% Plot method results
figure(3);
clf
plot(result);
axis equal


%%
trajectory = result.trajectory;
xx = result.trajectory.xx;
t = result.trajectory.t;
figure(1)
clf
plot(t, trajectory.c(1,:));

