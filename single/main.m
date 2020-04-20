%% Parameters
V = 20; %Reference constant speed
x0 = [0;0;0;V]; %initial condition
u0 = [0;0]; %initial input

%% Discretize and linearized model
Ts = 0.02;
[Ad,Bd,Cd,Dd,U,Y,X,DX] = obstacleVehicleModelDT(Ts,x0,u0);
dsys = ss(Ad,Bd,Cd,Dd,'Ts',Ts);
dsys.InputName = {'Throttle','Delta'};
dsys.StateName = {'X','Y','Theta','V'};
dsys.OutputName = dsys.StateName;

%% Road and Obstacle Info
lanes = 3;
laneWidth = 4;

obstacle = struct;
obstacle.Length = 5;
obstacle.Width = 2;
obstacle.X = 50;
obstacle.Y = 0;
obstacle.safeDistanceX = obstacle.Length;
obstacle.safeDistanceY = laneWidth;
obstacle = obstacleGenerateObstacleGeometryInfo(obstacle);
obstacle.DetectionDistance = 30;
f = obstaclePlotInitialCondition(x0,obstacle,laneWidth,lanes);

%% MPC Design
status = mpcverbosity('off');
mpcobj = mpc(dsys);
mpcobj.PredictionHorizon = 60;%25;
mpcobj.ControlHorizon = 2;%5;

% Hard constraints
%   Acceleration
mpcobj.ManipulatedVariables(1).RateMin = -0.2*Ts; 
mpcobj.ManipulatedVariables(1).RateMax = 0.2*Ts;
%   Steering speed
mpcobj.ManipulatedVariables(2).RateMin = -pi/30*Ts;
mpcobj.ManipulatedVariables(2).RateMax = pi/30*Ts;
%   Scaling of the two inputs
mpcobj.ManipulatedVariables(1).ScaleFactor = 2;
mpcobj.ManipulatedVariables(2).ScaleFactor = 0.2;

% Output tracking setting
mpcobj.Weights.OutputVariables = [0 30 0 1];
mpcobj.Model.Nominal = struct('U',U,'Y',Y,'X',X,'DX',DX);

% Road and Obstacle constraints
%   Road
E1 = [0 0];
F1 = [0 1 0 0]; 
G1 = laneWidth*lanes/2;
E2 = [0 0];
F2 = [0 -1 0 0]; 
G2 = laneWidth*lanes/2;
%   Obstacle placeholder
E3 = [0 0];
F3 = [0 -1 0 0]; 
G3 = laneWidth*lanes/2;
setconstraint(mpcobj,[E1;E2;E3],[F1;F2;F3],[G1;G2;G3],[1;1;0.1]);

%% Simulink model
mdl='mpc_ObstacleAvoidance';
open_system(mdl)
sim(mdl)