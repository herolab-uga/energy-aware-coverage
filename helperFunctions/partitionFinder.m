function [locations,locationsIdx, areaArray, verCellHandle, markerHandles, textHandles, frameObj] = partitionFinder( robotsPositions,  wts, res, densityFlag,robotariumBoundaries)
[x,y] =  meshgrid(robotariumBoundaries(1):res:robotariumBoundaries(2), robotariumBoundaries(3):res:robotariumBoundaries(4));
%[x,y] = meshgrid(-1.6:res:1.6,-1:res:1);
X = [x(:), y(:)];
distArray = zeros(1,size(robotsPositions,1));
areaArray = zeros(size(robotsPositions,1),1);
locations = cell(size(robotsPositions,1),1);
locationsIdx = cell(size(robotsPositions,1),1);
color = [1 0 0; 0 1 0;  0 0 0; 0 0 1; 0 1 1; 1 0 1; 0.9290 0.6940 0.1250; 0.8500 0.3250 0.0980; 0.4660 0.6740 0.1880; 0.6350 0.0780 0.1840	];
markerHandles = zeros(size(robotsPositions,1),1);
textHandles = zeros(size(robotsPositions,1),1);

% store values for robots' region to calculate centroid and
% density-weighted area
for xcount = 1:size(X,1)
    for r = 1:size(robotsPositions,1)
        distanceSq = ( robotsPositions(r,1)-X(xcount,1) )^2 + (robotsPositions(r,2)-X(xcount,2))^2 ;
        distArray(1,r) = (distanceSq) -  (wts(r));  % formula for power diagram
    end
    [minValue,minIndex] = min(distArray(1,:)); % find minimum dist of each row
    equalDistances = find(distArray == minValue); % find indices with equal distances
    for r = equalDistances
        locations{r} = [locations{r}, [X(xcount,1); X(xcount,2)]];
        locationsIdx{r} = [locationsIdx{r}, xcount];
    end
end

% visualize the voronoi partition
robotsName = repmat("", numel(robotsPositions), 1);
[v,c] = power_bounded(robotsPositions(:,1),robotsPositions(:,2), wts,[robotariumBoundaries(1) ,robotariumBoundaries(3);   robotariumBoundaries(1),robotariumBoundaries(4);   robotariumBoundaries(2),robotariumBoundaries(4);  robotariumBoundaries(2),robotariumBoundaries(3)]);
verCellHandle = zeros(size(robotsPositions,1),1);
for i = 1:size(robotsPositions,1)
    if(densityFlag)
        hold on;
        verCellHandle(i)  = patch(robotsPositions(i,1),robotsPositions(i,2),"black",'FaceColor',"white");
        set(verCellHandle(i), 'FaceAlpha', 0.01);
        set(verCellHandle(i), 'LineWidth', 6);
    else
        verCellHandle(i)  = patch(robotsPositions(i,1),robotsPositions(i,2),"white",'FaceColor',color(i,:)); % use color i  -- no robot assigned yet
        set(verCellHandle(i), 'FaceAlpha', 0.1);
    end
    set(verCellHandle(i), 'XData',v(c{i},1),'YData',v(c{i},2));
    robotsName(i) = num2str(i);
    hold on
end
axis tight;

for regionIdx = 1:numel(c)
    regionVertices = v(c{regionIdx}, :);
    areaArray(regionIdx) = polyarea(regionVertices(:, 1), regionVertices(:, 2));
end
end
