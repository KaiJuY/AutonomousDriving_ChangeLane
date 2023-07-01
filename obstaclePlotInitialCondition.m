function f = obstaclePlotInitialCondition(x0,obstacle,obstacleB,laneWidth,lanes)
% Create figure
f = figure;

% Plot the Ego vehicle.
carLength = 5;
carWidth = 2;
X0 = x0(1);
Y0 = x0(2);
plot(X0,Y0,'gx'); hold on; grid on;
rectangle('Position',[X0-carLength/2,Y0 - carWidth/2,carLength,carWidth],'EdgeColor','b');

% Plot the static obstacle.
plot(obstacle.X,obstacle.Y,'rx');
rectangle('Position',[obstacle.rrX,obstacle.rrY,obstacle.Length,obstacle.Width]);

% Plot the safe zone around obstacle.
rectangle('Position',[obstacle.rrSafeX,obstacle.rrSafeY,...
    (obstacle.safeDistanceX)*2,(obstacle.safeDistanceY)*2],...
    'LineStyle','-','EdgeColor','r');

% Plot the static obstacle.
plot(obstacleB.X,obstacleB.Y,'rx');
rectangle('Position',[obstacleB.rrX,obstacleB.rrY,obstacleB.Length,obstacleB.Width]);

% Plot the safe zone around obstacle.
rectangle('Position',[obstacleB.rrSafeX,obstacleB.rrSafeY,...
    (obstacleB.safeDistanceX)*2,(obstacleB.safeDistanceY)*2],...
    'LineStyle','-','EdgeColor',[0.6 0.1 0]);

% Plot the lanes.
X = [0;150;300];
Y = [2;2;2];
line(X,Y,'LineStyle','--','Color','b','LineWidth', 2 );
X = [0;150;300];
Y = [-2;-2;-2];
line(X,Y,'LineStyle','-','Color','b','LineWidth', 2 );
X = [0;150;300];
Y = [6;6;6];
line(X,Y,'LineStyle','-','Color','b','LineWidth', 2  );

% Reset the axis.
axis([0 100 -laneWidth*lanes/2 laneWidth*lanes/2]);
xlabel('X');
ylabel('Y');
title('Obstacle Avoidance Maneuver');


