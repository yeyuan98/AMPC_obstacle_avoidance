function detections = obstaclesDetect(x,obstacles,laneWidth)
% Detect when the vehicle sees each obstacle.
%#codegen
egoX = x(1);
egoY = x(2);
detections = {};
for i=1:length(obstacles)
    obstacle = obstacles{i};
    dist2Obstacle   = sqrt( (obstacle.X - egoX)^2 + (obstacle.Y - egoY)^2 );
    flagCloseEnough = (dist2Obstacle < obstacle.DetectionDistance);
    %flagInLane      = ( abs(obstacle.Y - egoY) < 2*laneWidth );
    flagInLane      = ( abs(obstacle.Y - egoY) < laneWidth );
    detection = ( flagCloseEnough && (egoX < obstacle.frSafeX) && flagInLane );
    detections{end+1} = detection;
end