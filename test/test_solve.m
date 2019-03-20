clear all

problem = prob.load('ship_viknes830.json', 'sjernaroyane.scenario.json');
gnc_settings = jsondecode(fileread('viknes830.gnc.json'));
problem.scenario.start_position = [7000; 1600];

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
xx = [result.trajectory(1).xx result.trajectory(2).xx];%[result.trajectory.xx];
t = [result.trajectory(1).t result.trajectory(2).t];%[result.trajectory.t];
figure(1)
clf
plot(t, xx(4,:));

for i=1:length(result.trajectory)
    if (result.trajectory(i).xx(4,1) < 0.1)
        i
    end
end