function obstacleNew = obstaclesPairCombine(obs1, obs2, laneWidth)
%OBSTACLESPAIRCOMBINE Combines two obstacles together and return one.
%   Combine strategy: regardless of relative speed, see them as still obstacles.
    obstacleNew = struct;
    obstacleNew.Length = abs(obs1.X - obs2.X) + (obs1.Length + obs2.Length)/2;
    obstacleNew.Width = (obs1.Width + obs2.Width)/2;
    obstacleNew.X = (obs1.X + obs2.X)/2;
    obstacleNew.Y = (obs1.Y + obs2.Y)/2;
    % Safe distance parameters
    obstacleNew.safeDistanceX = obstacleNew.Length;
    obstacleNew.safeDistanceY = laneWidth;
    % Generate obstacle geometry using the initial X,Y.
    obstacleNew = obstacleGenerateObstacleGeometryInfo(obstacleNew);
    obstacleNew.DetectionDistance = 30;
end