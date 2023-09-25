function [robot_alpha, robot_beta, weights, initial_energy] = getScenarioParams(scenarioName, N)
weights = ones(N,1);
initial_energy = ones(N,1)*100;
robot_alpha = ones(N,1);
robot_beta = ones(N,1);
switch scenarioName
    case 'Scenario1'
        disp(scenarioName);
    case 'Scenario2'
        robot_beta(4) = 5;
    case 'Scenario3'
        robot_alpha(4) = 2;
    case 'Scenario4'
        initial_energy(2) = 70;
    case 'Scenario5'
        initial_energy = [50;50;100;50];
        robot_alpha = [1;1;2;1];
    otherwise
        disp('Scenario not defined..');
end
end

