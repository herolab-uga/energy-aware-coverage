function [neighborsInfo,neighborsSharedBoundary,neighborsCount] = getNeighborsInformation_manualVoro(cellBoundaries)
%getNeighborsInformation Summary of this function goes here
%   Detailed explanation goes here

%pyversion;
pathtoPythonSearch = fileparts(which('SearchElem_ManualVoronoi3.py'));
if(count(py.sys.path,pathtoPythonSearch)) == 0
    insert(py.sys.path,int32(0),pathtoPythonSearch);
end

neighborsSharedBoundary = {};
neighborsInfo = {};
neighborsCount = {};
cellBoundaries;
for i =1:numel(cellBoundaries)
    %disp([" robot i",num2str(i)]);
    localNeighborsInfo = [];
    localNeighborsSharedBoundary = {};  % order of cell is robot#, BoundaryStartPoint, BoundaryEndPoint
    localneighborCount = 0;
    for j = 1:numel(cellBoundaries)
        if(i~=j)
            % Take it from python function
            cell_i = cellBoundaries{i};
            cell_j = cellBoundaries{j};
            if(~isempty(cell_i) & ~isempty(cell_j))
                boundaryPoints = py.SearchElem_ManualVoronoi3.getSharedBoundary( cell_i(:,1), cell_i(:,2) ,cell_j(:,1) , cell_j(:,2));
                boundaryPoints = cell(boundaryPoints);
                boundaryPoints = cell2mat(boundaryPoints);
                if(~isempty(boundaryPoints))
                    localneighborCount = localneighborCount + 1;
                    localNeighborsInfo(end+1) =  j;
                    if(j>i)
                        localNeighborsSharedBoundary = [localNeighborsSharedBoundary;[i],[j], [boundaryPoints(1),boundaryPoints(2),boundaryPoints(3),boundaryPoints(4)]]; % boundaryStartX, boundaryStartY, boundaryEndX, boundaryEndY
                    end
                end
            end
        end
    end
    neighborsInfo = [neighborsInfo; localNeighborsInfo];
    neighborsCount = [neighborsCount ; localneighborCount];
    neighborsSharedBoundary = [neighborsSharedBoundary;localNeighborsSharedBoundary];

end

end %% end function