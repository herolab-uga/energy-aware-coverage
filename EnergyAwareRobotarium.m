% scenario with different energy parameters
scenarios = ["Scenario1","Scenario2","Scenario3","Scenario4","Scenario5"];
initial_positions = [ -0.2, 0.2,0.2,-0.2;
                     -0.4, 0.4 ,-0.4, 0.4;
                      0,0,0,0];

% if helperFunctions and tp_utilities folder is not co-located with Main
% file, add the folder's path to matlab search path
addpath("tp_utilities/");
addpath("helperFunctions/")
% source locations for density function u1 and u2
u1 = [0.8,0.8];
u2 = [-0.8,-0.8];
% threshols distance for centroid
centroidThresholdDistance = 0.1;
% number of iterations
iterations = 1500;
% velocity of the robots
velocity = 0.1;
densityFlag = true;
% execute code for all 5 scenarios with a 15 second pause
for current_scenario = 1:numel(scenarios)
    executeController( scenarios(current_scenario), initial_positions, densityFlag, iterations, centroidThresholdDistance, velocity, u1, u2);
end