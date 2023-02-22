function [updated_weights] = UpdateWeights_eq10(cellBoundaries,density, neighborsCount, neighborsSharedBoundary, health, wts,Px,Py,Mass, k, alpha)
% UpdateWeights Summary of this function goes here
%   Detailed explanation goes heree
updated_weights = [];

for r = 1:numel(neighborsCount)  % for all the robots
    updated_wt_robot_r = 0;
    for i = 1:numel(neighborsSharedBoundary)  % get boundary edges that connect the ith robot to its neighbors
        currentSharedBoundary = neighborsSharedBoundary{i};
        if(currentSharedBoundary(1)== r || currentSharedBoundary(2) == r)
            boundaryEdges = [currentSharedBoundary(3),currentSharedBoundary(4),currentSharedBoundary(5),currentSharedBoundary(6)];    %Shared boundary
            d_ir = sqrt( (boundaryEdges(1)-boundaryEdges(3))^2 + (boundaryEdges(2)-boundaryEdges(4))^2 );        % length shared boundary
            weight_diff = 0;     
            if(currentSharedBoundary(1) == r)
                     weight_diff = (wts(r)-health(r)) - (wts(currentSharedBoundary(2))-health(currentSharedBoundary(2)));       % weight diff = wts(i)*h(i) - wts(j)*h(j) 
            else
                     weight_diff = (wts(r)-health(r)) - (wts(currentSharedBoundary(1))-health(currentSharedBoundary(1)));
            end
            updated_wt = weight_diff*d_ir;                       % weight_diff * boundary_length
            %updated_wt = weight_diff;
            updated_wt_robot_r = updated_wt_robot_r + updated_wt;  % add diff for all neighbors of r 
        end
    end
    
    updated_wt_robot_r = ((-alpha*k(r))/(2*Mass(r))) * updated_wt_robot_r; % multiply and divide by    alpha,k   and Mass
    updated_weights(end+1) = updated_wt_robot_r;
end


updated_weights = transpose(updated_weights);

end
