function [C_x, C_y, locationalCost_weighted, Mass] = getCentroidandCost(locationsInRobotRegion, locationIdxs, densityArray, currentRobotPositions, Wts, res, densityFlag)
%getCentroid Summary of this function goes here
%   Detailed explanation goes here
Mass = zeros(size(currentRobotPositions,1),1);
C_x = zeros(size(currentRobotPositions,1),1);
C_y = zeros(size(currentRobotPositions,1),1);
locationalCost_weighted = 0;
for r = 1:size(locationsInRobotRegion,1)
    Cx_r = 0;
    Cy_r = 0;
    Mass_r = 0;
    locationInRobotRegion = locationsInRobotRegion{r};
    robotRegionIdxs = locationIdxs{r};
    currentrobotLoc = currentRobotPositions(r,:);
    for pos = 1:size(locationInRobotRegion,2)
        if(densityFlag)
            densityArrayIdx = robotRegionIdxs(pos);
            dens = res*res*densityArray(densityArrayIdx);
        else
            dens = res*res;
        end
        Mass_r = Mass_r + dens;
        Cx_r = Cx_r + (dens*locationInRobotRegion(1,pos));
        Cy_r = Cy_r + (dens*locationInRobotRegion(2,pos));
        positionDiffSq = (locationInRobotRegion(1,pos)-currentrobotLoc(:,1) )^2 + (locationInRobotRegion(2,pos)-currentrobotLoc(:,2))^2;
        locationalCost_weighted = locationalCost_weighted + 0.5*(dens*(positionDiffSq - Wts(r)));
    end

    Cx_r = Cx_r/Mass_r;
    Cy_r = Cy_r/Mass_r;
    C_x(r) = Cx_r;
    C_y(r) = Cy_r;
    Mass(r) = Mass_r;
end
end




