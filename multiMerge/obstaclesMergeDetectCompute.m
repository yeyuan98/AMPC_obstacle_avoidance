function [E,F,G] = obstaclesMergeDetectCompute(x,obstacles,laneWidth,lanes)
%OBSTACLESMERGEDETECTCOMPUTE Detects obstacles, merges them to minimize interaction, and computes MPC constraint matrices.
mergeThres = 25; %[m]

egoX = x(1);
egoY = x(2);
detections = [];
for i=1:length(obstacles)
    obstacle = obstacles{i};
    dist2Obstacle   = sqrt( (obstacle.X - egoX)^2 + (obstacle.Y - egoY)^2 );
    flagCloseEnough = (dist2Obstacle < obstacle.DetectionDistance);
    %flagInLane      = ( abs(obstacle.Y - egoY) < 2*laneWidth );
    flagInLane      = ( abs(obstacle.Y - egoY) < laneWidth );
    detection = ( flagCloseEnough && (egoX < obstacle.frSafeX) && flagInLane );
    detections(end+1) = detection;
end
% We then see whether detected obstacles can be merged together
detections = logical(detections);
activeObstacles = obstacles(detections);
mergedObstacles = {};
%   This preliminary version only supports 2 obstacles moving in X and same Y.
if length(activeObstacles) == 2
    if abs(activeObstacles{1}.X - activeObstacles{2}.X) < mergeThres
        mergedObstacles{end+1} = obstaclesPairCombine(activeObstacles{1},activeObstacles{2}, laneWidth);
        detections = logical([1,0]);
        % merge happened, then detections change to only one, and the first
        % element in mergedObstacles.
    end
else
    mergedObstacles = obstacles; % no merge, then detections don't need to change and mergedObstacles is just obstacles
end

%Next, use the obstaclesCompute function to get the matrices.
%   Before that, convert logical detections to cell array
detectionsCell = {};
for i=1:length(detections)
    detectionsCell{end+1} = detections(i);
end

[E,F,G] = obstaclesComputeCustomConstraint(x, detectionsCell, mergedObstacles, laneWidth, lanes);
end