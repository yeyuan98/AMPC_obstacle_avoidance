function obstacles = obstaclesCreate(XY0, XYSpeed, laneWidth)
%OBSTACLECREATE creates a cell array of obstacle structs w/ identical base settings and adjustable initial position and speed.
obstacles = {};
for i=1:length(XY0)
    XYi = XY0{i};
    XYSpeedi = XYSpeed{i};
    % both input should be identical length cell arrays w/ [2;1] vector elements.
    obstacle = struct;
    obstacle.Length = 5;
    obstacle.Width = 2;
    obstacle.X = XYi(1);
    obstacle.Y = XYi(2);
    % Movement parameters
    obstacle.X0 = XYi(1);
    obstacle.Y0 = XYi(2);
    obstacle.XYSpeed = XYSpeedi;
    % Safe distance parameters
    obstacle.safeDistanceX = obstacle.Length;
    obstacle.safeDistanceY = laneWidth;
    % Generate obstacle geometry using the initial X,Y.
    obstacle = obstacleGenerateObstacleGeometryInfo(obstacle);
    obstacle.DetectionDistance = 100;
    obstacles{i} = obstacle;
end
end

