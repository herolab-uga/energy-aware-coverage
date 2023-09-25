function [weightChange] = UpdateWeights_DepletionRatioController(kValues,mass,currentEnergyLevel,initialEnergy,currentWts,energySpentPerMeter)
% Cal
weightChange = zeros(numel(currentEnergyLevel),1);
for i = 1:numel(currentEnergyLevel) 
    weight_change_r = 0;
    for j = 1:numel(currentEnergyLevel)  
          if(i~=j)         
             energyRatio = (energySpentPerMeter(j)/energySpentPerMeter(i)) * (initialEnergy(i)/initialEnergy(j));
             weights_Ratio = ( currentWts(i) / currentWts(j) ) ;
             weight_diff = energyRatio - weights_Ratio ;   
             weight_change_r = weight_change_r + weight_diff;  % add diff for all 
          end         
    end
        weightChange(i) = weight_change_r./numel(currentEnergyLevel);
end
end
