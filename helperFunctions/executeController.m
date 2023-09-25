function [] = executeController(selectedScenario, initialRobotsPositions, densityFlag, robotariumIterations, centroidThresholdDistance, velocity, u1, u2)
warning('off','all')
set(0, 'DefaultFigureVisible', 'on');
N = size(initialRobotsPositions,2);
velocity = ones(1,N)*velocity;
res = 0.1;
% for each Scenario, get energyParams from file
[robotsAlpha,robotsBeta, updatedWeights, initialRobotsEnergy] = getScenarioParams( selectedScenario,N );
robotarium = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initialRobotsPositions);
timeStep = robotarium.time_step;
robotariumBoundaries = robotarium.boundaries;
unicycle_barrier_certificate = create_uni_barrier_certificate();
currentRobotsEnergy = initialRobotsEnergy;
energySpentPerMeter = zeros(N,1);
energySpentAtLastTimeStep = currentRobotsEnergy;
distToCentroid = zeros(N,1);
distanceTravelledArray = zeros(N,1);
weightValuesCell = cell(N,1);
areaCell = cell(N,1);
energyDepletionCell = cell(N,1);
KVALUES =  zeros(N,1);
densityArray = zeros(1,1);
iter = cell(1,1);
locationalCostCell_DRC = cell(1,1);
energyDiff = zeros(N,1);

[si_to_uni_dyn, uni_to_si_states] = create_si_to_uni_mapping();
markers = {'o', 's', 'd', '^', 'v', '*', 'x', '+','pentagram','_'};
color = [1 0 0; 0 1 0;  0 0 0; 0 0 1; 0 1 1; 1 0 1; 0.9290 0.6940 0.1250; 0.8500 0.3250 0.0980; 0.4660 0.6740 0.1880; 0.6350 0.0780 0.1840	];
% Calculate and show density plot if densityFlag = true
if(densityFlag)
    [x,y] =  meshgrid(robotariumBoundaries(1):res:robotariumBoundaries(2), robotariumBoundaries(3):res:robotariumBoundaries(4));
    X = [x(:) y(:)];
    sigma = 0.6  * eye(2);
    covInv = inv(sigma);
    densityArray = zeros(size(X,1),1);
    constant_term = 4/(1* pi * sqrt(det(sigma)));
    q = [X(:,1),X(:,2)];
    diff1 = q - u1;
    diff2 = q - u2;
    for i=1:numel(densityArray)
        exponent1 = -0.5 .* (diff1(i,:) * covInv * diff1(i,:)');
        exponent2 = -0.5 .* (diff2(i,:) * covInv * diff2(i,:)');
        densityArray(i) = constant_term * (exp(exponent1) + exp(exponent2));
    end
    density = reshape(densityArray, size(x) );
    %figure;
    h = pcolor(x,y,density);
    set(h,'EdgeColor',"None")
end

% get robots positions from robotarium
currentRobotsPositions_unicycle = robotarium.get_poses();
currentRobotsPositions = uni_to_si_states(currentRobotsPositions_unicycle);
positionAtLastTimeStep = transpose(currentRobotsPositions);
font_size = determine_font_size(robotarium, 0.13);
for r = 1:N
    robot_caption = sprintf('%d', r);
    robot_labels{r} = text(currentRobotsPositions(1,r)-0.12, currentRobotsPositions(2,r)+0.12, robot_caption, 'FontSize', font_size, 'FontWeight', 'bold');
end
start_time = tic; %The start time to compute time elapsed.
iteration_caption = sprintf('Iteration %d', 0);
time_caption = sprintf('Total Time Elapsed %0.2f', toc(start_time));
iteration_label = text(-1.5, -0.8, iteration_caption, 'FontSize', font_size, 'Color', 'r', 'FontWeight', 'bold');
time_label = text(-1.5, -0.9, time_caption, 'FontSize', font_size, 'Color', 'r', 'FontWeight', 'bold');
robotarium.step();
count = 1;
font_size = determine_font_size(robotarium, 0.05);
tic;
% robotaium iteration time is 0.033sec. We'll perform calculations after
% every 20th iteration
for i= 1:robotariumIterations
    currentRobotsPositions_unicycle = robotarium.get_poses();
    if(i==1 || mod(i,21)==0)
        iteration_caption = sprintf('Iteration %d', i);
        time_caption = sprintf('Total Time Elapsed %0.2f', toc(start_time));
        iteration_label.String = iteration_caption;
        time_label.String = time_caption;        
        iteration_label.FontSize = font_size;
        time_label.FontSize = font_size;
        currentRobotsPositions = uni_to_si_states(currentRobotsPositions_unicycle);
        if(i>1) 
             for patches = 1:numel(verCellHandle)
                vorPatch = verCellHandle(patches,1);
                delete(vorPatch);
             end
        end
        [locationsForPartitions, locationsIdx, area, verCellHandle, markerHandles,textHandles] = partitionFinder(transpose(currentRobotsPositions), updatedWeights,res, densityFlag, robotariumBoundaries);
        [C_x, C_y, locationalCost_weighted,Mass] = getCentroidandCost(locationsForPartitions, locationsIdx, densityArray ,  transpose(currentRobotsPositions),updatedWeights,res,densityFlag);     
        locationalCostCell_DRC{1} = [locationalCostCell_DRC{1},locationalCost_weighted];
        if(i > 1)
            energyDiff = zeros(N,1);
            energyDiff = abs(energySpentAtLastTimeStep - energySpentPerMeter);
            energyDiff = round(energyDiff,1);
            if(any(energyDiff)- 0 > 0.1)
                initialRobotsEnergy = currentRobotsEnergy;
                disp("Energy Spent Value for some robot has changed");
            end        
            weightsAtLastTimeStep = updatedWeights;
            % Apply weight controller
            weightsChangeArray =  UpdateWeights_DepletionRatioController(KVALUES, area,currentRobotsEnergy,(initialRobotsEnergy),updatedWeights,energySpentPerMeter);
            updatedWeights = weightsAtLastTimeStep +  (weightsChangeArray);
        end
        dxi = zeros(2,N);
        robotsPositions_diff = (  [C_x,C_y] - transpose(currentRobotsPositions(1:2,:)) );
        % for each robot r, apply lloyd algorithm to calculate velocity
        for r = 1:N
            KVALUES(r) = velocity(r);
            distToCentroid(r) = abs(sqrt( (currentRobotsPositions(1,r)-C_x(r) )^2 + (currentRobotsPositions(2,r)-C_y(r))^2 ));
            vel = (robotsPositions_diff(r,:)./distToCentroid(r)).*KVALUES(r);
            dxi(:,r) = (vel);
        end
        dxu = si_to_uni_dyn(dxi, currentRobotsPositions_unicycle); 
        dxu = unicycle_barrier_certificate(dxu, currentRobotsPositions_unicycle);
        for r = 1:N
            robot_labels{r}.Position = currentRobotsPositions(1:2, r) + [-0.12;0.12];
            distanceTravelledArray(r) = sqrt(( positionAtLastTimeStep(r,1)- currentRobotsPositions(1,r) )^2 + (positionAtLastTimeStep(r,2)-currentRobotsPositions(2,r))^2);
            energyAtLastTimeStep = currentRobotsEnergy(r);
            %%%%%%%%%%%%%%%%       get robot's depleted Energy
            energySpentPerMeter(r) = getDepletedEnergy(currentRobotsEnergy(r),distanceTravelledArray(r),robotsAlpha(r),robotsBeta(r), timeStep*20);
            currentRobotsEnergy(r) =  ( energyAtLastTimeStep - energySpentPerMeter(r));
            energyDepletionCell{r} = [energyDepletionCell{r};currentRobotsEnergy(r)];
        end      
        % Update Iteration and Time marker
        positionAtLastTimeStep = transpose(currentRobotsPositions);
        robotarium.set_velocities(1:N, dxu);
        iter{1} = [iter{1},count];
        count = count + 1;
        for r = 1:N
             weightValuesCell{r} = [weightValuesCell{r},[updatedWeights(r,:)]];
            if densityFlag
                areaCell{r} = [areaCell{r};Mass(r,:)];
            else
                areaCell{r} = [areaCell{r};area(r,:)];
            end
        end
        if (any(currentRobotsEnergy - 0 < 5) ||  all(distToCentroid- 0 < centroidThresholdDistance))
            break;
        end
    end
        robotarium.step();
end
hold off;
figure;
for r =  1:N
    plot(iter{1}, weightValuesCell{r} ,Marker = markers(1,r),Color = color(r,:))
    hold on;
end
legend('Robot1', 'Robot2','Robot3', 'Robot4','Robot5', 'Robot6'); %
ylabel("weights")
xlabel("Time (sec)")         
hold off;

figure;
for r =  1:N
    plot(iter{1},energyDepletionCell{r},Marker = markers(1,r),Color=color(r,:));
    hold on;
end
legend('Robot1', 'Robot2','Robot3', 'Robot4','Robot5', 'Robot6');
ylabel("EnergyLevel (J)");
xlabel("Time (sec)");
hold off;


figure;
for r =  1:N        
    plot(iter{1},areaCell{r},Marker = markers(1,r),Color=color(r,:));
    hold on;
end
legend('Robot1', 'Robot2','Robot3', 'Robot4','Robot5', 'Robot6'); %
if densityFlag
    ylabel("\phi-Weighted Area");
else
    ylabel("Area (m^2)");
end
xlabel("Time (sec)");
hold off;

figure;
plot(iter{1},locationalCostCell_DRC{1},Color=color(1,:),Marker=markers(1));
legend('DRC'); 
ylabel("Cost")
xlabel("Time (sec)");
hold off;

robotarium.debug();
pause(5);
close("all")
end



% Font Size Helper Function to scale size with figure window
% Input: robotarium instance, desired height of the font in meters
function font_size = determine_font_size(robotarium_instance, font_height_meters)

% Get the size of the robotarium figure window in point units
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Pixels');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);

% Determine the ratio of the font height to the y-axis
font_ratio = (font_height_meters)/(robotarium_instance.boundaries(4) -...
    robotarium_instance.boundaries(3));

% Determine the font size in points so it fits the window. cursize(4) is
% the hight of the figure window in points.
font_size = cursize(4) * font_ratio;

end
