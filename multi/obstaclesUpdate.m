function obstaclesNew = obstaclesUpdate(obstacles,k,T,endTime)
%OBSTACLESUPDATE Update obstacle positions and rebuild geometry
obstaclesNew = {};
for i=1:length(obstacles)
    obstacle = obstacles{i};
    obstacle.X = obstacle.X0 + k/length(T)*endTime*obstacle.XYSpeed(1);
    obstacle.Y = obstacle.Y0 + k/length(T)*endTime*obstacle.XYSpeed(2);
    obstacle = obstacleGenerateObstacleGeometryInfo(obstacle);
    obstaclesNew{i} = obstacle;
end
end

