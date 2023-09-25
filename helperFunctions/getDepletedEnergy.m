function [depletedEnergy] = getDepletedEnergy(lastEnergy,distanceTravelled,robotAlpha,robotBeta,timeStep)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

depletedEnergy =  (robotAlpha*timeStep) + (robotBeta*distanceTravelled);

end
