function [detection, obstacleXY] = obstacleDetect_moving(x,time,obstacle,laneWidth)
% Detect when the vehicle sees an obstacle.
%#codegen
egoX = x(1);
egoY = x(2);
dist2Obstacle   = sqrt( (obstacle.X + obstacle.XSpeed*time - egoX)^2 + (obstacle.Y - egoY)^2 );
flagCloseEnough = (dist2Obstacle < obstacle.DetectionDistance);
flagInLane      = ( abs(obstacle.Y - egoY) < 2*laneWidth );
detection = ( flagCloseEnough && (egoX < obstacle.frSafeX) && flagInLane );
obstacleXY = [obstacle.X + obstacle.XSpeed*time,obstacle.Y];