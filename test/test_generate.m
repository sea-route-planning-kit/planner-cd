clear all

problem = prob.load('ship_viknes830.json', 'scenario.json');

%% Calculate
result = cd.generate(problem);

%% Plot method results
figure(3);
clf
plot(result);
axis equal