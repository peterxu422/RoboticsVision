% HW3- Team 23
%Frederik Clinckemaillie - fec2109
%Peter Xu - px2117

function hw2_Team23(serPort)
% Simple program for the iRobot Create or the associated simulator.  This
% program will make the robot go straight until a wall is reached, and then
% follow that wall until it arrives within a certain threshold of its
% original value.

% Input:
% serPort - Serial port object, used for communicating over bluetooth

    % Set constants for this program
    maxDuration= 1200;  % Max time to allow the program to run (s)
    maxTimeSinceLastUpdate = 60; %Max time to allow without any updates to the map
    % Initialize loop variables
    tTimeSinceLastUpdate= tic;        % Time limit marker
    v= 0.2;               % Forward velocity (m/s)
    w = 0;              %Initialize angular velocity to 0
    
    %Odometry Variables
    currentPosX = 0.0;
    currentPosY = 0.0;
    currentRot = 0.0;
    
    %resetting the Sensors
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);
    
    SetFwdVelAngVelCreate(serPort,v,w);
    %Used to calculate the amount of distance the robot traveled inside the
    %current box.  This is used because we only allow a box status to be
    %updated after the robot has traveled at least half its width to ensure
    %we do not get too many false empty boxes in the map.
    prevBoxCornerX = 0;
    prevBoxCornerY = 0;
    newBoxPointX = 0;
    newBoxPointY = 0;

    posFig = figure; %% % Create handle for position figure
    axis equal; 
	time = zeros(maxDuration, 1);	% Create arrays for time and orientation to be printed later.
	orient = zeros(maxDuration, 1);
	tctr = 1;
    mapgrid = zeros(52); %%Assuming a maximum roomsize of 10m x 10m
    origX = 26;
    origY = 26;
    % Enter main loop
    while toc(tTimeSinceLastUpdate) < maxTimeSinceLastUpdate 
        
        boxCornerX = floor(currentPosX/0.4)* 0.4 ;
        boxCornerY = floor(currentPosY/0.4)* 0.4 ;
        
        if ~(prevBoxCornerX == boxCornerX) || ~(prevBoxCornerY == boxCornerY)
            newBoxPointX = currentPosX;
            newBoxPointY = currentPosY;
            prevBoxCornerX = boxCornerX;
            prevBoxCornerY = boxCornerY;
        end
      
        SetFwdVelAngVelCreate(serPort,v,w);
        
 
        % Check bump sensors (ignore wheel drop sensors)
        [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster ...
            BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        bumped= BumpRight || BumpLeft || BumpFront;
        
        % Halt forward motion and turn only if bumped
        if bumped
            v = 0;
            SetFwdVelAngVelCreate(serPort, v, 0);
            
            ang = 0;
            w = 0.2* (floor(rand(1,1) -0.5) * 2 + 1);
            SetFwdVelAngVelCreate(serPort,v,w); % Turn clockwise
            ang= rand(1,1)* pi;

            objRot = currentRot;
            if BumpLeft
                objRot = currentRot + sqrt(3) * pi / 2;
            elseif BumpLeft
                objRot = currentRot - sqrt(3) * pi / 2;
            end
            objPosX = currentPosX + cos(objRot) * 0.2;
            objPosY = currentPosY + sin(objRot) * 0.2;

            boxCornerX = floor(objPosX/0.4)* 0.4 ;
            boxCornerY = floor(objPosY/0.4)* 0.4 ;            
            tTimeSinceLastUpdate= tic;
            
            mapCellX = origX + floor(objPosX/0.4);
            mapCellY = origY - floor(objPosY/0.4);
            
            if(mapgrid(mapCell,mapCellX) == 0)
                mapgrid(mapCellY,mapCellX)= -1;
                rectangle('position',[boxCornerX boxCornerY 0.4 0.4], 'FaceColor','r');
            end
            [currentPosX, currentPosY, currentRot] = updateCurrent(serPort,currentPosX, currentPosY, currentRot);        
            
            % Wait for turn to complete 
            [currentPosX, currentPosY, currentRot] =angleTurn(serPort, ang, currentPosX, currentPosY, currentRot);
        end
        
        
            mapCellX = origX + floor(currentPosX/0.4);
            mapCellY = origY - floor(currentPosY/0.4);
        
        v = 0.2;
        w = 0;
        [currentPosX, currentPosY, currentRot] = updateCurrent(serPort,currentPosX, currentPosY, currentRot);
        figure(posFig);
        distTraveledInBox = sqrt((currentPosY - newBoxPointY)^2 + (currentPosX - newBoxPointX)^2);
        
        if distTraveledInBox > 0.2 && mapgrid(mapCellY,mapCellX) == 0
            
            tTimeSinceLastUpdate= tic;
            mapgrid(mapCellY,mapCellX) = 1;
            rectangle('position',[boxCornerX boxCornerY 0.4 0.4], 'FaceColor','g');
        end
        hold on; 
        pause(0.1);
    end
    % Stop robot motion
    v= 0;
    w= 0;
    SetFwdVelAngVelCreate(serPort,v,w);
    [currentPosX, currentPosY, currentRot] = updateCurrent(serPort,currentPosX, currentPosY, currentRot);
    [tctr, time, orient] = recordOrient(tctr, time, orient, tTimeSinceLastUpdate, currentRot); %%
    figure(posFig); scatter(currentPosX,currentPosY); hold on; %%
    %%
    figure(posFig);
    title('Robot Position'); xlabel('y position'); ylabel('x position');

    figure; 
    plot(time(1:tctr-1), orient(1:tctr-1));
    title('Robot Orientation'); xlabel('time (sec)'); ylabel('Angle (rad)');
end    


function [currentPosX, currentPosY, currentRot] = updateCurrent(serPort,posX, posY, rot)
% Updates the current Robot position
%
% Input:
% serPort - Serial port object, used for communicating over bluetooth
% posX - X component of the previous position of the robot
% posY - Y component of the previous position of the robot
% rot - Previous rotation of the robot
%
% Output:
% currentPosX - Updated X component of the current position of the robot
% currentPosY - Updated Y component of the current position of the robot
% currentRot - Updated current rotation of the robot

    currentRot = rot;
    dist = DistanceSensorRoomba(serPort);
    currentPosX = posX + cos(rot) * dist;
    currentPosY = posY + sin(rot) * dist;
        
    %Plots the current x,y values of the robot position.
    %scatter(currentPosX,currentPosY); hold on;

end


function [posX, posY, rot] =  angleTurn(serPort, ang, posX, posY, rot)
    angTurned = 0;
    
    while angTurned < ang
        
        rotation = AngleSensorRoomba(serPort);
        if rotation < - 1 * pi 
            rotation = rotation + 2 * pi;
        elseif rotation > pi
            rotation = rotation - 2 * pi;
        end
        angTurned = angTurned+abs(rotation);
        rot = rot + rotation;
        [posX, posY, rot] =updateCurrent(serPort,posX, posY, rot);
        pause(0.1);
    end
     rotation = AngleSensorRoomba(serPort);
     rot = rot + rotation;
     [posX, posY, rot] =updateCurrent(serPort,posX, posY, rot);
end

function [tctr, time, orient] = recordOrient(tctr, time, orient, tStart, currentRot)
    time(tctr) = toc(tStart);
    orient(tctr) = currentRot;
    fprintf('time: %4.2f, rot: %4.2f\n', time(tctr), orient(tctr));
    tctr = tctr + 1;   
end
