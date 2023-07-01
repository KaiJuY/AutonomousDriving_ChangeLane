%% Assume all the states are measurable. At the nominal operating point, the
% ego car drives east at a constant speed of |20| meters per second.
V = 20;
x0 = [0; 0; 0; V]; 
u0 = [0; 0];
MPC_StartControl_Dis = 15;
Reference_Noob_Driver = load('nonsafe_ref.mat');
%Reference_Noob_Driver = load('safe_Ref.mat');
Reference_Noob_Slope = load('nonsafe_Slope.mat');
%Reference_Noob_Slope = load('safe_Slopee.mat');
Reference_Noob_Intercept = load('nonsafe_Intercrpt.mat');
%Reference_Noob_Intercept = load('safe_Intercrptt.mat');
%%
% Discretize the continuous-time model using the zero-order holder method
% in the |obstacleVehicleModelDT| function.
Ts = 0.02;
[Ad,Bd,Cd,Dd,U,Y,X,DX] = obstacleVehicleModelDT(Ts,x0,u0);
dsys = ss(Ad,Bd,Cd,Dd,Ts=Ts);
dsys.InputName = {'Throttle','Delta'};
dsys.StateName = {'X','Y','Theta','V'};
dsys.OutputName = dsys.StateName;

%% Road and Obstacle Information
% In this example, assume that:
%
% * The road is straight and has three lanes.
% * Each lane is four meters wide.
% * The ego car drives in the middle of the center lane when not passing.
% * Without losing generality, the ego car passes an obstacle only from the
% left (fast) lane.
%
lanes = 3;
laneWidth = 4;

%% 
% The obstacle in this example is a nonmoving object in the middle of the
% center lane with the same size as the ego car.
obstacle = struct;
obstacle.Length = 5;
obstacle.Width = 1;
obstacleB = obstacle;

%% 
% Place the obstacle |50| meters down the road.
obstacle.X = 50;
obstacle.Y = 0;
obstacleB.X = 150;
obstacleB.Y = 4;

%%
% Create a virtual safe zone around the obstacle so that the ego car does
% not get too close to the obstacle when passing it. The safe zone is
% centered on the obstacle and has a:
%
% * Length equal to two car lengths
% * Width equal to two lane widths
% Normal distribution p = 0.8 then Length * 1.28
obstacle.safeDistanceX = obstacle.Length*1.28;
obstacle.safeDistanceY = obstacle.Width*2;
obstacle = obstacleGenerateObstacleGeometryInfo(obstacle);

obstacleB.safeDistanceX = obstacleB.Length*1.28;
obstacleB.safeDistanceY = obstacleB.Width*2;
obstacleB = obstacleGenerateObstacleGeometryInfo(obstacleB);

%%
% In this example, assume that the lidar device can detect an obstacle |30|
% meters in front of the vehicle.
obstacle.DetectionDistance = 30;
obstacleB.DetectionDistance = obstacle.DetectionDistance;

%%
% Plot the following at the nominal condition:
%
% * Ego car - Green dot with black boundary
% * Horizontal lanes - Dashed blue lines
% * Obstacle - Red |x| with black boundary
% * Safe zone - Dashed red boundary.
%
f = obstaclePlotInitialCondition(x0, obstacle, obstacleB, laneWidth, lanes);
%% MPC Design at the Nominal Operating Point
% Design a model predictive controller that can make the ego car maintain
% a desired velocity and stay in the middle of the center lane.
status = mpcverbosity("off");
mpcobj = mpc(dsys);

%% 
% The prediction horizon is |25| steps, which is equivalent to 0.5 seconds.
mpcobj.PredictionHorizon = 60;%25;
mpcobj.ControlHorizon = 2;%5;

%%
% To prevent the ego car from accelerating or decelerating too quickly, add
% a hard constraint of 0.2 (m/s^2) on the throttle rate of change.
mpcobj.ManipulatedVariables(1).RateMin = -0.2*Ts; 
mpcobj.ManipulatedVariables(1).RateMax = 0.2*Ts;

%%
% Similarly, add a hard constraint of 6 degrees per second on the steering
% angle rate of change.
mpcobj.ManipulatedVariables(2).RateMin = -pi/30*Ts;
mpcobj.ManipulatedVariables(2).RateMax = pi/30*Ts;

%%
% Scale the throttle and steering angle by their respective operating
% ranges.
mpcobj.ManipulatedVariables(1).ScaleFactor = 2;
mpcobj.ManipulatedVariables(2).ScaleFactor = 0.2;

%%
% Since there are only two manipulated variables, to achieve zero
% steady-state offset, you can choose only two outputs for perfect
% tracking. In this example, choose the Y position and velocity by setting
% the weights of the other two outputs (X and theta) to zero. Doing so lets
% the values of these other outputs float.
mpcobj.Weights.OutputVariables = [0 30 0 1];

%% 
% Update the controller with the nominal operating condition. For a
% discrete-time plant:
%
% * |U = u0|
% * |X = x0|
% * |Y = Cd*x0 + Dd*u0|
% * |DX = Ad*X0 + Bd*u0 - x0|
%
mpcobj.Model.Nominal = struct(U=U,Y=Y,X=X,DX=DX);

%% Specify Mixed I/O Constraints for Obstacle Avoidance Maneuver
% There are different strategies to make the ego car avoid an obstacle on
% the road. For example, a real-time path planner can compute a new path
% after an obstacle is detected and the controller follows this path.
%
% In this example, use a different approach that takes advantage of the
% ability of MPC to handle constraints explicitly. When an obstacle is
% detected, it defines an area on the road (in terms of constraints) that
% the ego car must not enter during the prediction horizon. At the next
% control interval, the area is redefined based on the new positions of
% the ego car and obstacle until passing is completed.
%
% To define the area to avoid, use the following mixed input/output
% constraints:
%
%   E*u + F*y <= G
% 
% where |u| is the manipulated variable vector and |y| is the output
% variable vector. You can update the constraint matrices |E|, |F|, and |G|
% when the controller is running.

%%
% The first constraint is an upper bound on $y$ ($y \le 6$ on this
% three-lane road).
E1 = [0 0];
F1 = [0 1 0 0]; 
G1 = 6;

%%
% The second constraint is a lower bound on $y$ ($y \ge -6$ on this
% three-lane road).
E2 = [0 0];
F2 = [0 -1 0 0]; 
G2 = 2;

%%
% The third constraint is for obstacle avoidance. Even though no obstacle
% is detected at the nominal operating condition, you must add a "fake"
% constraint here because you cannot change the dimensions of the
% constraint matrices at run time. For the fake constraint, use a
% constraint with the same form as the second constraint.
E3 = [0 0];
F3 = [0 -1 0 0]; 
G3 = laneWidth*lanes/2;

%%
% Specify the mixed input/output constraints in the controller using the
% |setconstraint| function.
setconstraint(mpcobj,[E1;E2;E3],[F1;F2;F3],[G1;G2;G3],[1;1;0.1]);

%%
% Use a constant reference signal.
refSignal = [0 0 0 V];

%%
% Initialize plant and controller states.
x = x0;
u = u0;
egoStates = mpcstate(mpcobj);

%%
% The simulation time is |10| seconds.
T = 0:Ts:15;

%%
% Log simulation data for plotting.
saveSlope = zeros(length(T),1);
saveIntercept = zeros(length(T),1);
ympc = zeros(length(T),size(Cd,1));
umpc = zeros(length(T),size(Bd,2));

%%
% Run the simulation.
for k = 1:length(T)
    % Obtain new plant model and output measurements for interval |k|.
    [Ad,Bd,Cd,Dd,U,Y,X,DX] = obstacleVehicleModelDT(Ts,x,u);
    measurements = Cd * x + Dd * u;
    ympc(k,:) = measurements';
    obstacle.X = obstacle.X + 0.125;
    %obstacle.Y = obstacle.Y + 0.005;
    obstacleB.X = obstacleB.X + 0.125;
    obstacle = obstacleGenerateObstacleGeometryInfo(obstacle);
    obstacleB = obstacleGenerateObstacleGeometryInfo(obstacleB);
    txt = num2str(k*0.02) + "s";
        if rem(k,50) == 0
            %Plot the Ego Car
            plot(x(1),x(2),'rx');
            rectangle('Position',[x(1)-5/2,x(2)-1/2,obstacle.Length,obstacle.Width],'FaceColor','b');
            text((x(1)-5/2), (x(2)-1/2), txt, 'VerticalAlignment','bottom');
        end
        if rem(k,200) == 0
            % Plot the static obstacle.
            plot(obstacle.X,obstacle.Y,'rx');
            rectangle('Position',[obstacle.rrX,obstacle.rrY,obstacle.Length,obstacle.Width]);
            % Plot the static obstacle.
            plot(obstacleB.X,obstacleB.Y,'rx');
            rectangle('Position',[obstacleB.rrX,obstacleB.rrY,obstacleB.Length,obstacleB.Width]);
            
            % Plot the safe zone around obstacle.
            rectangle('Position',[obstacle.rrSafeX,obstacle.rrSafeY,...
                (obstacle.safeDistanceX)*2,(obstacle.safeDistanceY)*2],...
                'LineStyle','--','EdgeColor','r');
            text(obstacle.rrSafeX,obstacle.rrSafeY, txt, 'VerticalAlignment','bottom');
            % Plot the safe zone around obstacle.
            rectangle('Position',[obstacleB.rrSafeX,obstacleB.rrSafeY,...
                (obstacleB.safeDistanceX)*2,(obstacleB.safeDistanceY)*2],...
                'LineStyle','--','EdgeColor',[0.7 0.1 0]);
            text(obstacleB.rrSafeX,obstacleB.rrSafeY, txt, 'VerticalAlignment','bottom');
        end
    % Determine whether the vehicle sees the obstacle, and update the mixed
    % I/O constraints when obstacle is detected.
    % If Ego didn't Pass the obstacle safe area
    if(k == 1)
        detection = obstacleDetect(x,obstacle,laneWidth);
        [E,F,G,saveSlope(k),saveIntercept(k)] = ...
            obstacleComputeCustomConstraint(x,detection,obstacle,laneWidth,lanes); 
    end
    if(obstacle.flSafeX - x(1) > 0)
    detection = obstacleDetect(x,obstacle,laneWidth);
        %Here to mix driver & controller slope
        if ( (x(1) > obstacle.rlSafeX - MPC_StartControl_Dis) && (x(2) < obstacle.rlSafeY +4))
        [E,F,G,saveSlope(k),saveIntercept(k)] = ...
            obstacleComputeCustomConstraint(x,detection,obstacle,laneWidth,lanes); 
        else
        saveSlope(k) = Reference_Noob_Slope.saveSlope(k);
        saveIntercept = Reference_Noob_Intercept.saveIntercept(k);
        end
    else
        detection = obstacleDetect(x,obstacleB,laneWidth);
        %Here to mix driver & controller slope
        if ( (x(1) > obstacleB.rrSafeX -MPC_StartControl_Dis) && (x(2) > obstacleB.rrSafeY -4))
        [E,F,G,saveSlope(k),saveIntercept(k)] = ...
            obstacleComputeCustomConstraint(x,detection,obstacleB,laneWidth,lanes); 
        else
        saveSlope(k) = Reference_Noob_Slope.saveSlope(k);
        saveIntercept = Reference_Noob_Intercept.saveIntercept(k);
        end
   

    end
    


    % Prepare new plant model and nominal conditions for adaptive MPC.
    newPlant = ss(Ad,Bd,Cd,Dd,Ts=Ts);
    newNominal = struct(U=U,Y=Y,X=X,DX=DX);
    
    % Prepare new mixed I/O constraints.
    options = mpcmoveopt;
    options.CustomConstraint = struct(E=E,F=F,G=G);
    
    % Compute optimal moves using the updated plant, nominal conditions,
    % and constraints.
    [u,Info] = mpcmoveAdaptive(mpcobj,egoStates,newPlant,newNominal,...
        measurements,refSignal,[],options);
    umpc(k,:) = u';
    
    % Update the plant state for the next iteration |k+1|.
    x = Ad * x + Bd * u;
end

mpcverbosity(status);

%% Analyze Results
% Plot the trajectory of the ego car (black line) and the third mixed
% I/O constraints (dashed green lines) during the obstacle avoidance
% maneuver.
%figure(f)
%for k = 1:length(saveSlope)
 %   X = [0;100;200];
  %  Y = saveSlope(k)*X + saveIntercept(k);
   % if (Y ~= -6)
    %        if(rem(k,10) == 0)
     %           line(X,Y,LineStyle="--",Color="g")
      %      end
   % end
%end   
plot(ympc(:,1),ympc(:,2),"-k");
plot(Reference_Noob_Driver.ympc(:,1),Reference_Noob_Driver.ympc(:,2),"-k",'Color','g');
axis([0 ympc(end,1) -2 6]) % reset axis

f2 = figure;
figure(f2);
hold on; grid on;
plot(T,Reference_Noob_Driver.ympc(:,3),"-k",'Color','g','DisplayName','Driver');
plot(T,ympc(:,3),"-k",'DisplayName','MPC');
axis([0 T(end) -0.15 0.15]) % reset axis
xlabel('Time');
ylabel('Angle');
title('Steering Angle');
%% Simulate Controller in Simulink
% Open the Simulink model. The obstacle avoidance system contains multiple
% components:
%
% * Plant Model Generator: Produce new plant model and nominal values.
% * Obstacle Detector: Detect obstacle (lidar sensor not included).
% * Constraint Generator: Produce new mixed I/O constraints.
% * Adaptive MPC: Control obstacle avoidance maneuver.

%mdl = "mpc_ObstacleAvoidance";
%open_system(mdl)
%sim(mdl)
