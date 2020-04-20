%% Parameters
V = 20; %Reference constant speed
x0 = [0;0;0;V]; %initial condition
u0 = [0;0]; %initial input

%% Discretize and linearized model, at Nominal
Ts = 0.02;
[Ad,Bd,Cd,Dd,U,Y,X,DX] = obstacleVehicleModelDT(Ts,x0,u0);
dsys = ss(Ad,Bd,Cd,Dd,'Ts',Ts);
dsys.InputName = {'Throttle','Delta'};
dsys.StateName = {'X','Y','Theta','V'};
dsys.OutputName = dsys.StateName;

%% Road and Obstacle Info
lanes = 3;
laneWidth = 4;

%   Multiple moving obstacles
obstacles = obstaclesCreate({[50;0],[70;0]},{[5;0],[5;0]}, laneWidth);
%obstacles = obstaclesCreate({[50;0]},{[5;0]}, laneWidth);
figure;
f = obstaclesPlotState(x0,obstacles,laneWidth,lanes);

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
%   Obstacle placeholders
soften_road = 1;
soften_obstacle = 0.1;
E3 = [];
F3 = [];
G3 = [];
V3 = [];
for i=1:length(obstacles)
    E3 = [E3;[0 0]];
    F3 = [F3;[0 -1 0 0]]; 
    G3 = [G3;laneWidth*lanes/2];
    V3 = [V3;soften_obstacle];
end
setconstraint(mpcobj,[E1;E2;E3],[F1;F2;F3],[G1;G2;G3],[soften_road;soften_road;V3]);

%% Define reference signal, initialize plant and controller states
refSignal = [0 0 0 V];
x = x0;
u = u0;
egoStates = mpcstate(mpcobj);
% Simulation time
endTime = 10;
T = 0:Ts:endTime;
% Data logging
saveSlope = zeros(length(T),1);
saveIntercept = zeros(length(T),1);
ympc = zeros(length(T),size(Cd,1));
umpc = zeros(length(T),size(Bd,2));


%% Running simulation
figure(f)
plotN = 100;
for k = 1:length(T)
    % Obtain new plant model and output measurements for interval |k|.
    [Ad,Bd,Cd,Dd,U,Y,X,DX] = obstacleVehicleModelDT(Ts,x,u);
    measurements = Cd * x + Dd * u;
    ympc(k,:) = measurements';
    
    % Determine whether the vehicle sees the obstacle, and update the mixed
    % I/O constraints when obstacle is detected.
    % Update obstacle position
    obstacles = obstaclesUpdate(obstacles,k,T,endTime);
        %obstacle.X = obstacle.X0 + k/length(T)*endTime*obstacle.XYSpeed(1);
        %obstacle.Y = obstacle.Y0 + k/length(T)*endTime*obstacle.XYSpeed(2);
        % Rebuild obstacle geometry
        %obstacle = obstacleGenerateObstacleGeometryInfo(obstacle);
    % Update detections
    [E,F,G] = ...
        obstaclesMergeDetectCompute(x,obstacles,laneWidth,lanes); 
   
    % Prepare new plant model and nominal conditions for adaptive MPC.
    newPlant = ss(Ad,Bd,Cd,Dd,'Ts',Ts);
    newNominal = struct('U',U,'Y',Y,'X',X,'DX',DX);
    
    % Prepare new mixed I/O constraints.
    options = mpcmoveopt;
    options.CustomConstraint = struct('E',E,'F',F,'G',G);
    
    % Compute optimal moves using the updated plant, nominal conditions,
    % and constraints.
    [u,Info] = mpcmoveAdaptive(mpcobj,egoStates,newPlant,newNominal,...
        measurements,refSignal,[],options);
    umpc(k,:) = u';
    
    % Plot the new ego vehicle and obstacle positions
    if (mod(k,plotN) == 0)
       obstaclesPlotState([x(1) x(2)],obstacles, laneWidth, lanes)
    end
    % Update the plant state for the next iteration |k+1|.
    x = Ad * x + Bd * u;
end

mpcverbosity(status);


%% Finish up plotting
plot(ympc(:,1),ympc(:,2),'-k'); % draw ego vehicle path
axis([0 ympc(end,1) -laneWidth*lanes/2 laneWidth*lanes/2]) % reset axis
hold off