% HW1- Team 23
%Frederik Clinckemaillie - fec2109
%Peter Xu - px2117

function hw2_Team23(serPort)
% Simple program for the iRobot Create or the associated simulator.  This
% program will make the robot go straight until a wall is reached, and then
% follow that wall until it arrives within a certain threshold of its
% original value.

% Input:
% serPort - Serial port object, used for communicating over bluetooth

    goalPointX = 4.0;
    goalPointY = 0.0;
    wallFollowing = 0;
    finished = 0;

    % Set constants for this program
    maxDuration= 1200;  % Max time to allow the program to run (s)
    
    % Initialize loop variables
    tStart= tic;        % Time limit marker
    distSansBump= 0;    % Distance traveled without hitting obstacles (m)
    v= 0.2;               % Forward velocity (m/s)
    w = 0;              %Initialize angular velocity to 0
    
    %Odometry Variables
    wallFound = 0;
    currentPosX = 0.0;
    currentPosY = 0.0;
    currentRot = 0.0;
    wallStartPosX = 0.0;
    wallStartPosY = 0.0;
    wallStartRot = 0.0;
    previousPosX = 0.0;
    previousPosY = 0.0;
    
    %resetting the Sensors
    dist = DistanceSensorRoomba(serPort);
    ang = AngleSensorRoomba(serPort);
    
    %Starting point threshold
    leftStartArea = 0;
    inStartArea = 0;
    leaveAreaDistThresh = 0.3;
    enterAreaDistThresh = 0.2;
    
    %Plots the position of the robot. (assumes a start at (0,0) with
    %orientation angle 0.
       %scatter(currentPosX,currentPosY); hold on;
    SetFwdVelAngVelCreate(serPort,v,w);
	
	
	%posFig = figure; %% % Create handle for position figure
	%scatter(currentPosX, currentPosY);
	time = zeros(maxDuration, 1);	% Create arrays for time and orientation to be printed later.
	orient = zeros(maxDuration, 1);
	tctr = 1;
    %travelDist(serPort, 0.5, 2);
    %pause(5.0);
    [cmd, val] = textread('commands.txt', '%s %f')
    
    for i=1:length(cmd)
       if strcmp(cmd(i), 'move')
          v = 0.35;
          travelDist(serPort, v, val(i));
       elseif strcmp(cmd(i), 'turn')
           angval = val(i);
           if val(i) > 0
               w = 0.2;
           else
               angval = angval*-1;
               w = -0.2;
           end
               
           SetFwdVelAngVelCreate(serPort, 0, w);
           ang = angval * pi/180;
           [currentPosX, currentPosY, currentRot] =angleTurn(serPort, ang, currentPosX, currentPosY, currentRot);
       end
    end
    
    
    % Enter main loop
%     while toc(tStart) < maxDuration && ~finished
%         
%         
%         fprintf('WallFollowing: %d\n', wallFollowing);
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         % Go towards goal if no obstacle is found
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         if(wallFollowing == 0)
%             wallFound = 0;
%             leftStartArea = 0;
%             ang = atan(goalPointY/goalPointX) - currentRot;
%             if(ang > 0)
%                 SetFwdVelAngVelCreate(serPort, 0, 0.2);
%             else
%                 SetFwdVelAngVelCreate(serPort, 0, -0.2);
%                 ang = abs(ang);
%             end
%             while ang > 2 * pi
%                 ang  = ang - 2 * pi
%             end
%             
%             [currentPosX, currentPosY, currentRot] =angleTurn(serPort, ang, currentPosX, currentPosY, currentRot);
% 			[tctr, time, orient] = recordOrient(tctr, time, orient, tStart, currentRot); %%
%             while(~wallFollowing && ~finished)
%                 bumped = 0;
%                 SetFwdVelAngVelCreate(serPort,v,w);
%                 [bumped, currentPosX, currentPosY, currentRot] = bumpCheckReact(serPort, currentPosX, currentPosY, currentRot);
%                 if bumped
%                     wallFollowing = 1;
%                 end
%                 [currentPosX, currentPosY, currentRot] = updateCurrent(serPort,currentPosX, currentPosY, currentRot);
%                 dist = sqrt((currentPosX - goalPointX) ^ 2 + (currentPosY - goalPointY) ^ 2);
% 				[tctr, time, orient] = recordOrient(tctr, time, orient, tStart, currentRot); %%
% 				figure(posFig); scatter(currentPosX,currentPosY); hold on; %%
%                 if dist < enterAreaDistThresh
%                     finished = 1;
%                     fprintf('Goal point Reached\n');
%                 end;
%                 pause(0.1)    
%             end
%         else
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         % Follow the wall if an obstacle is found
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             SetFwdVelAngVelCreate(serPort, 0, 0);
%             % Start Robot moving
%             % Check for and react to bump sensor readings
%             [bumped, currentPosX, currentPosY, currentRot] = bumpCheckReact(serPort, currentPosX, currentPosY, currentRot);
% 
%             WallSensor = WallSensorReadRoomba(serPort);
% 
%             % If an obstacle is found, the robot's current position is saved to
%             % remember the start point of the wall following program
%             if ~wallFound 
%                 currentRot = currentRot + AngleSensorRoomba(serPort); 
%                 dist = DistanceSensorRoomba(serPort);
%                 currentPosX = currentPosX + cos(currentRot) * dist;   wallStartPosX = currentPosX;
%                 currentPosY = currentPosY + sin(currentRot) * dist;   wallStartPosY = currentPosY;
%                 %scatter(currentPosX,currentPosY); hold on;
% 				
% 				[tctr, time, orient] = recordOrient(tctr, time, orient, tStart, currentRot); %%
%                 wallFound = 1;
%             else
%                
%                 %When the robot leaves the starting area, as defined by the
%                 %leaveStartAreaDistThresh variable, a flag is turned on,
%                 %telling the robot it now has to start checking for when it
%                 %re-enters the starting area.
%                 if ~leftStartArea && pdist([currentPosX, currentPosY; wallStartPosX, wallStartPosY], 'euclidean') > leaveAreaDistThresh
%                     leftStartArea = 1;
%                 end
% 
%                 %While the wall is detected using the while sensor, we go
%                 %forward, checking for results on the bump sensors.
%                 while WallSensor && wallFollowing
%                     fprintf('1\n');
%                     SetFwdVelAngVelCreate(serPort, v, 0);
%                     pause(0.1);
%                     %The current position is then checked against the starting
%                     %area.
%                     [currentPosX, currentPosY, currentRot] = updateCurrent(serPort,currentPosX, currentPosY, currentRot);
% 					[tctr, time, orient] = recordOrient(tctr, time, orient, tStart, currentRot); %%
% 					figure(posFig); scatter(currentPosX,currentPosY); hold on; %%
%                     wallFollowing = checkFoundLine(wallFollowing,currentPosX, currentPosY, leftStartArea, goalPointX, goalPointY, enterAreaDistThresh);
%                     if(wallFollowing)
%                         WallSensor = WallSensorReadRoomba(serPort);
% 
%                         %Correct itself a bit and see if the wall is still there
%                         [bump, currentPosX, currentPosY, currentRot] = bumpCheckReact(serPort, currentPosX, currentPosY, currentRot);
%                         SetFwdVelAngVelCreate(serPort, v, 0);
%                         wallFollowing = checkFoundLine(wallFollowing,currentPosX, currentPosY, leftStartArea, goalPointX, goalPointY, enterAreaDistThresh);
%                        % If WallSensor is lost, correct itself a bit and see if the wall is still there
%                         % ~bump condition ensures that correction only takes place
%                         % when it moved too far from the wall.
%                         if(wallFollowing && WallSensor == 0 && ~bump)
%                             SetFwdVelAngVelCreate(serPort, 0, -0.2);
%                             ang = pi/22;
%                            [currentPosX, currentPosY, currentRot] =angleTurn(serPort, ang, currentPosX, currentPosY, currentRot);
% 						   [tctr, time, orient] = recordOrient(tctr, time, orient, tStart, currentRot); %%
%                             WallSensor = WallSensorReadRoomba(serPort);
%                             travelDist(serPort, v, 0.05);
%                         end
%                     end
%                 end
% 
%                 %Turn around Corner
%                 display(WallSensor);
%                 [bump, currentPosX, currentPosY, currentRot] = bumpCheckReact(serPort, currentPosX, currentPosY, currentRot);
%                 SetFwdVelAngVelCreate(serPort, v, 0);
%                 wallFollowing = checkFoundLine(wallFollowing,currentPosX, currentPosY, leftStartArea, goalPointX, goalPointY, enterAreaDistThresh);
%                 fprintf('3\n');
%                 if(wallFollowing && WallSensor == 0 && ~bump)
%                     fprintf('2\n');
%                     SetFwdVelAngVelCreate(serPort, v, 0);
%                     travelDist(serPort, v, 0.13);
%                     [currentPosX, currentPosY, currentRot] = updateCurrent(serPort,currentPosX, currentPosY, currentRot);
% 					
% 					[tctr, time, orient] = recordOrient(tctr, time, orient, tStart, currentRot); %%
%                     figure(posFig); scatter(currentPosX,currentPosY); hold on; %%
% 					
%                     wallFollowing = checkFoundLine(wallFollowing,currentPosX, currentPosY, leftStartArea, goalPointX, goalPointY, enterAreaDistThresh);
%                     
%                     SetFwdVelAngVelCreate(serPort, 0, -0.2);
%                     [currentPosX, currentPosY, currentRot] = updateCurrent(serPort,currentPosX, currentPosY, currentRot);
% 					[tctr, time, orient] = recordOrient(tctr, time, orient, tStart, currentRot); %%
% 					figure(posFig); scatter(currentPosX,currentPosY); hold on; %%
% 
%                     ang = pi/2;
%                     [currentPosX, currentPosY, currentRot] =angleTurn(serPort, ang, currentPosX, currentPosY, currentRot);
%                 end
%             end
% 
%            [bump, currentPosX, currentPosY, currentRot] = bumpCheckReact(serPort, currentPosX, currentPosY, currentRot);
% 
% 
%             WallSensor = WallSensorReadRoomba(serPort);
%             while(wallFollowing && WallSensor == 0 && wallFound && ~bump)
% fprintf('4\n');
%                 travelDist(serPort, v, 0.25);
% 
%                 previousPosX = currentPosX;   currentPosY = currentPosY;
%                 [currentPosX, currentPosY, currentRot] = updateCurrent(serPort,currentPosX, currentPosY, currentRot);
% 				[tctr, time, orient] = recordOrient(tctr, time, orient, tStart, currentRot); %%
% 				figure(posFig); scatter(currentPosX,currentPosY); hold on; %%
%                 
%                 wallFollowing = checkFoundLine(wallFollowing,currentPosX, currentPosY, leftStartArea, goalPointX, goalPointY, enterAreaDistThresh);
%               
%                 SetFwdVelAngVelCreate(serPort, 0, -0.2);
% 
%                 previousPosX = currentPosX;   currentPosY = currentPosY;
%                 [currentPosX, currentPosY, currentRot] = updateCurrent(serPort,currentPosX, currentPosY, currentRot);
% 				
% 				[tctr, time, orient] = recordOrient(tctr, time, orient, tStart, currentRot); %%
% 				figure(posFig); scatter(currentPosX,currentPosY); hold on; %%
% 				
%                 wallFollowing = checkFoundLine(wallFollowing,currentPosX, currentPosY, leftStartArea, goalPointX, goalPointY, enterAreaDistThresh);
%                 if(wallFollowing)
% 
%                     ang = pi/17;
%                     [currentPosX, currentPosY, currentRot] =angleTurn(serPort, ang, currentPosX, currentPosY, currentRot);
%                 
%                     WallSensor = WallSensorReadRoomba(serPort);
%                     [bump, currentPosX, currentPosY, currentRot] = bumpCheckReact(serPort, currentPosX, currentPosY, currentRot);
%                     wallFollowing = checkFoundLine(wallFollowing,currentPosX, currentPosY, leftStartArea, goalPointX, goalPointY, enterAreaDistThresh);
%                 end
%                 
%             end
%             % Briefly pause to avoid continuous loop iteration
%             pause(0.1)
%         end
% 
%     end
%         
%     
%     % Specify output parameter
%     finalRad= v/w;
%     
%     % Stop robot motion
%     v= 0;
%     w= 0;
%     SetFwdVelAngVelCreate(serPort,v,w);
%     previousPosX = currentPosX;   currentPosY = currentPosY;
%     [currentPosX, currentPosY, currentRot] = updateCurrent(serPort,currentPosX, currentPosY, currentRot);
% 	[tctr, time, orient] = recordOrient(tctr, time, orient, tStart, currentRot); %%
% 	figure(posFig); scatter(currentPosX,currentPosY); hold on; %%	
% 	
% 	%%
%     figure(posFig);
%     title('Robot Position'); xlabel('y position'); ylabel('x position');
%     
%     figure; 
%     plot(time(1:tctr-1), orient(1:tctr-1));
%     title('Robot Orientation'); xlabel('time (sec)'); ylabel('Angle (rad)');
end    

function [bumped, currentPosX, currentPosY, currentRot] = bumpCheckReact(serPort, currentPosX, currentPosY, currentRot)
% Check bump sensors and steer the robot away from obstacles if necessary
%
% Input:
% serPort - Serial port object, used for communicating over bluetooth
% currentPosX - X component of the current position of the robot
% currentPosY - Y component of the current position of the robot
% currentRot - Current rotation of the robot
%
% Output:
% bumped - Boolean, true if bump sensor is activated
% currentPosX - Updated X component of the current position of the robot
% currentPosY - Updated Y component of the current position of the robot
% currentRot - Updated current rotation of the robot

    % Check bump sensors (ignore wheel drop sensors)
    [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster ...
        BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    bumped= BumpRight || BumpLeft || BumpFront;
    
    % Halt forward motion and turn only if bumped
    if bumped
        %AngleSensorRoomba(serPort);     % Reset angular odometry
        %v= 0.45;       % Forward velocity
        %w= v2w(v);  % Angular velocity
        v = 0;
        SetFwdVelAngVelCreate(serPort, v, 0);
        
        % Turn away from obstacle
        ang = 0;
        if BumpFront
            w = 0.2;
            WallSensor = WallSensorReadRoomba(serPort);
            %If WallSensor == 1 and BumpFront, then it's in a corner and
            %must turn CC pi/2
            if WallSensor
                SetFwdVelAngVelCreate(serPort, v, w);
                ang=pi/2;
            else
                %w = v2w(v);
                SetFwdVelAngVelCreate(serPort,v,w);  % Turn counter-clockwise
                ang= pi/2;                          % Turn further
            end
        elseif BumpLeft
            w = 0.2;
           SetFwdVelAngVelCreate(serPort,v,w); % Turn clockwise
            ang= 3*pi/4;
        elseif BumpRight
            w = 0.2;
           SetFwdVelAngVelCreate(serPort,v,w);  % Turn counter-clockwise
            ang= pi/19;  % Angle to turn
        end
        
        [currentPosX, currentPosY, currentRot] = updateCurrent(serPort,currentPosX, currentPosY, currentRot);        
        
        % Wait for turn to complete 
        [currentPosX, currentPosY, currentRot] =angleTurn(serPort, ang, currentPosX, currentPosY, currentRot);
    end
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


function wallFollowing = checkFoundLine(wallFollowing,currentPosX, currentPosY, leftStartArea, endX, endY, distThresh)
% Check to see if the robot is within a certain threshold of the start
% area.  If it is, the current robot position is compared to the previous
% robot position to see if the robot is still getting closer to the start
% point.  If it is found that the robot is getting further away from the 
%
% Input:
% currentPosX - X component of the current position of the robot
% currentPosY - Y component of the current position of the robot
% previousPosX - X component of the previous position of the robot
% previousPosY - Y component of the previous position of the robot
% wallStartPosX - X component of the starting position of the robot after
% it hit a wall
% wallStartPosY - Y component of the starting position of the robot afterf
% it hit a wall
% inStartArea - Boolean marking whether the robot is in the start area.
% leftStartArea-  Boolean marking whether the robot has once left the start
% area.
%
% Output:
% bumped -Boolean marking whether the robot is in the start area.
distance = abs( -endX*currentPosY  + endY * currentPosX)/ sqrt(endX^2 + endY^2);
    if distance < distThresh && leftStartArea
        wallFollowing = 0;
    end
    fprintf('dist: %f wallfoll %d\n', distance, wallFollowing);
end

function [posX, posY, rot] =  angleTurn(serPort, ang, posX, posY, rot)
    angTurned = 0;
    fprintf('Angle turning: %f Goal: %f \n', angTurned, ang);
    while angTurned < ang
        
        rotation = AngleSensorRoomba(serPort);
        if rotation < - 1 * pi 
            rotation = rotation + 2 * pi;
        elseif rotation > pi
            rotation = rotation - 2 * pi;
        end
        angTurned = angTurned+abs(rotation);
        fprintf('Angle turning: %f Goal: %f rotation %f \n', angTurned, ang,rotation);
        rot = rot + rotation;
        [posX, posY, rot] =updateCurrent(serPort,posX, posY, rot);
        pause(0.1);
    end
end

function [tctr, time, orient] = recordOrient(tctr, time, orient, tStart, currentRot)
    time(tctr) = toc(tStart);
    orient(tctr) = currentRot;
    fprintf('time: %4.2f, rot: %4.2f\n', time(tctr), orient(tctr));
    tctr = tctr + 1;   
end

function w= v2w(v)
% Calculate the maximum allowable angular velocity from the linear velocity
%
% Input:
% v - Forward velocity of Create (m/s)
%
% Output:
% w - Angular velocity of Create (rad/s)
    
    % Robot constants
    maxWheelVel= 0.5;   % Max linear velocity of each drive wheel (m/s)
    robotRadius= 0.2;   % Radius of the robot (m)
    
    % Max velocity combinations obey rule v+wr <= v_max
    w= (maxWheelVel-v)/robotRadius;
end