function [E,F,G,constraintSlope,constraintIntercept] = obstacleComputeCustomConstraint(x,detection,obstacle,laneWidth,lanes)
%% Compute custom constraints for the obstacle.

%#codegen
egoX = x(1);
egoY = x(2);
TA = egoX + 15;
nonsafe = 0;
if nonsafe == 0
    if obstacle.Y > laneWidth*lanes/6 %upper than middle line use rear right corner
        Safe_X = obstacle.rrSafeX;
        Safe_Y = obstacle.rrSafeY;
        Safe_AX = obstacle.frSafeX;
        CurrentLine_Center = laneWidth*lanes/3;
        CrossLine_Center = -laneWidth*lanes/2;
    else
        Safe_X = obstacle.rlSafeX;
        Safe_Y = obstacle.rlSafeY;
        Safe_AX = obstacle.flSafeX;
        CurrentLine_Center = -laneWidth*lanes/2;
        CrossLine_Center = laneWidth*lanes/3;
    end
elseif nonsafe == 1
        if obstacle.Y > laneWidth*lanes/6 %upper than middle line use rear right corner
        Safe_X = obstacle.rrSafeX;
        Safe_Y = obstacle.rrY;
        Safe_AX = obstacle.frSafeX;
        CurrentLine_Center = laneWidth*lanes/3;
        CrossLine_Center = -laneWidth*lanes/2;
    else
        Safe_X = obstacle.rlSafeX;
        Safe_Y = obstacle.rlY;
        Safe_AX = obstacle.flSafeX;
        CurrentLine_Center = -laneWidth*lanes/2;
        CrossLine_Center = laneWidth*lanes/3;
    end
end
% Compute constraints only if an obstacle is detected. Otherwsie, set
% constraint to lower road boundary (the inactive constraint).
if detection
    slope =  ( (Safe_Y - egoY)/(Safe_X - egoX) );
    % Didn't Pass rear side
    if (egoX <= Safe_X)
            %Obstacle is on the upper line need to check ego > Obstacle
            %rear right side
            plot(Safe_X,Safe_Y,'gx');
            if obstacle.Y > laneWidth*lanes/6 
                %Need to caculate constrainSlope & constraintIntercept base
                %on Safe_X & Safe_Y
                if egoY > Safe_Y
                constraintSlope = tan(atan2(slope,1));
                constraintIntercept = Safe_Y - constraintSlope*Safe_X;
                %Need to caculate constrainSlope & constraintIntercept base
                %on Cross Line Center
                else
                slope = ( (CrossLine_Center - egoY) / (TA - egoX) );
                constraintSlope = tan(atan2(slope,1));
                constraintIntercept = CrossLine_Center - constraintSlope * TA;
                end
            %Obstacle is on the lower line need to check ego < Obstacle
            else
                %Need to caculate constrainSlope & constraintIntercept base
                %on Safe_X & Safe_Y
                if egoY < Safe_Y
                constraintSlope = tan(atan2(slope,1));
                constraintIntercept = Safe_Y - constraintSlope*Safe_X;
                %Need to caculate constrainSlope & constraintIntercept base
                %on Cross Line Center
                else
                slope = ( (CrossLine_Center - egoY) / (TA - egoX) );
                constraintSlope = tan(atan2(slope,1));
                constraintIntercept = CrossLine_Center - constraintSlope * TA;
                end
            end
     %Beside the obstacle
    elseif( (egoX > Safe_X) && (egoX <= Safe_AX) )
            %Seems Keep in the cross line
            slope = ( (CrossLine_Center - egoY) / (TA - egoX) );
            constraintSlope = tan(atan2(slope,1));
            constraintIntercept = CrossLine_Center - constraintSlope * TA;
    %Passed the obstacle
    else
            %Seems Keep in the cross line
            slope = ( (CrossLine_Center - egoY) / (TA - egoX) );
            constraintSlope = tan(atan2(slope,1));
            constraintIntercept = CrossLine_Center - constraintSlope * TA;
    end
else
    constraintSlope = 0;
    if(egoY > 2)
    constraintIntercept = laneWidth*lanes/3;
    else
    constraintIntercept = -laneWidth*lanes/2;
    end
end

%% Define constraint matrices.
E = [0 0;0 0;0 0];
F = [0 1 0 0;0 -1 0 0;constraintSlope -1 0 0]; 
G = [laneWidth*lanes/2;laneWidth*lanes/2;-1*constraintIntercept];
