clear all

scenario = simulator.scenario.load('scenario.json');
ship = simulator.ship.load('ship_viknes830.json');

%% Calculate
result = cd.generate(ship, scenario);

%% Plot method results
figure(3);
clf
plot(result);
axis equal