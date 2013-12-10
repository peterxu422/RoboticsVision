% HW1- Team 23
%Frederik Clinckemaillie - fec2109
%Peter Xu - px2117

function hw1_Team23(serPort)
% Simple program for the iRobot Create or the associated simulator.  This
% program will make the robot go straight until a wall is reached, and then
% follow that wall until it arrives within a certain threshold of its
% original value.

% Input:
% serPort - Serial port object, used for communicating over bluetooth

    % Set constants for this program
    maxDuration= 1200;  % Max time to allow the program to run (s)
    
    % Initialize loop variables
    tStart= tic;        % Time limit marker
    distSansBump= 0;    % Distance traveled without hitting obstacles (m)
    angTurned= 0;       % Angle turned since turning radius increase (rad)
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
    leaveStartAreaDistThresh = 1.3;
    enterStartAreaDistThresh = 0.5;
    
    %Plots the position of the robot. (assumes a start at (0,0) with
    %orientation angle 0.
       %scatter(currentPosX,currentPosY); hold on;
    
    % Start robot moving
    SetFwdVelAngVelCreate(serPort,v,w)
    
    posFig = figure; 
    set(gcf,'numbertitle','off','name','Robot Position') 
    %xlabel('y position'); ylabel('x position'); title('Robot Position');
    
    axis([-5, 5, -5, 5]);
    %axis manual;
    s = scatter(currentPosX, currentPosY);
    time = zeros(maxDuration, 1);
    orient = zeros(maxDuration, 1);
    tctr = 1;
    % Enter main loop
    while (toc(tStart) < maxDuration) && (inStartArea == 0)
        % Start Robot moving
        % Check for and react to bump sensor readings
        
        [bumped, currentPosX, currentPosY, currentRot] = bumpCheckReact(serPort, currentPosX, currentPosY, currentRot);
        [currentPosX, currentPosY, currentRot] = updateCurrent(serPort, currentPosX, currentPosY, currentRot);
        [tctr, time, orient] = recordOrient(tctr, time, orient, tStart, currentRot);
        
        %fprintf('currentPosX: %4.2f, currentPosY: %4.2f\n', currentPosX, currentPosY);
        
        WallSensor = WallSensorReadRoomba(serPort);
        
        % If an obstacle is found, the robot's current position is saved to
        % remember the start point of the wall following program
        if bumped && wallFound == 0
            currentRot = currentRot + AngleSensorRoomba(serPort); 
            dist = DistanceSensorRoomba(serPort);
            currentPosX = currentPosX + cos(currentRot) * dist;   wallStartPosX = currentPosX;
            currentPosY = currentPosY + sin(currentRot) * dist;   wallStartPosY = currentPosY;
            fprintf('currentPosX: %4.2f, currentPosY: %4.2f, currentRot: %4.2f\n', currentPosX, currentPosY, currentRot);
            figure(posFig);
            scatter(currentPosX,currentPosY); hold on;
            [tctr, time, orient] = recordOrient(tctr, time, orient, tStart, currentRot);
            
            wallFound = 1;
        end
        
        if wallFound
            %When the robot leaves the starting area, as defined by the
            %leaveStartAreaDistThresh variable, a flag is turned on,
            %telling the robot it now has to start checking for when it
            %re-enters the starting area.
            if ~leftStartArea && pdist([currentPosX, currentPosY; wallStartPosX, wallStartPosY], 'euclidean') > leaveStartAreaDistThresh
                leftStartArea = 1;
            end
            
            %While the wall is detected using the while sensor, we go
            %forward, checking for results on the bump sensors.
            while WallSensor
                SetFwdVelAngVelCreate(serPort, v, 0);
                pause(0.1);

               %The previous position is saved, and the position is then updated.
               %The current position is then checked against the starting
               %area.
               previousPosX = currentPosX;   previousPosY = currentPosY;
               [currentPosX, currentPosY, currentRot] = updateCurrent(serPort,currentPosX, currentPosY, currentRot);             
               inStartArea = checkStartArea(currentPosX, currentPosY, previousPosX, previousPosY, wallStartPosX, wallStartPosY, inStartArea, leftStartArea);
               fprintf('currentPosX: %4.2f, currentPosY: %4.2f, currentRot: %4.2f\n', currentPosX, currentPosY, currentRot);
               
               figure(posFig); scatter(currentPosX,currentPosY); hold on;
               [tctr, time, orient] = recordOrient(tctr, time, orient, tStart, currentRot);
               WallSensor = WallSensorReadRoomba(serPort);

                %Correct itsel  f a bit and see if the wall is still there
                [bump, currentPosX, currentPosY, currentRot] = bumpCheckReact(serPort, currentPosX, currentPosY, currentRot);
                      if ~inStartArea && leftStartArea && pdist([currentPosX, currentPosY; wallStartPosX, wallStartPosY], 'euclidean') < enterStartAreaDistThresh
                        inStartArea = 1;
                      end
               % If WallSensor is lost, correct itself a bit and see if the wall is still there
                % ~bump condition ensures that correction only takes place
                % when it moved too far from the wall.
                if(WallSensor == 0 && ~bump)
                    SetFwdVelAngVelCreate(serPort, 0, -0.2);
                    ang = pi/22;
                    angTurned = 0;
                    while angTurned < ang
                       rotation = AngleSensorRoomba(serPort);
                       angTurned = angTurned+abs(rotation);
                       currentRot = currentRot + rotation;
                       [currentPosX, currentPosY, currentRot] =updateCurrent(serPort,currentPosX, currentPosY, currentRot);
                       [tctr, time, orient] = recordOrient(tctr, time, orient, tStart, currentRot);
                       pause(0.1);
                    end
                    
                    WallSensor = WallSensorReadRoomba(serPort);
                    travelDist(serPort, v, 0.05);
                end
                
            end
            
            %Turn around Corner
            display(WallSensor);
            [bump, currentPosX, currentPosY, currentRot] = bumpCheckReact(serPort, currentPosX, currentPosY, currentRot);
            inStartArea = checkStartArea(currentPosX, currentPosY, previousPosX, previousPosY, wallStartPosX, wallStartPosY, inStartArea, leftStartArea);

            if(WallSensor == 0 && ~bump)
              travelDist(serPort, v, 0.13);
               previousPosX = currentPosX;   previousPosY = currentPosY;
               [currentPosX, currentPosY, currentRot] = updateCurrent(serPort,currentPosX, currentPosY, currentRot);
               
               fprintf('currentPosX: %4.2f, currentPosY: %4.2f, currentRot: %4.2f\n', currentPosX, currentPosY, currentRot);
               
               figure(posFig); scatter(currentPosX,currentPosY); hold on;
               
               inStartArea = checkStartArea(currentPosX, currentPosY, previousPosX, previousPosY, wallStartPosX, wallStartPosY, inStartArea, leftStartArea);
                             
               SetFwdVelAngVelCreate(serPort, 0, -0.2);
               [currentPosX, currentPosY, currentRot] = updateCurrent(serPort,currentPosX, currentPosY, currentRot);
               
               ang = pi/2;
               angTurned = 0;
               while angTurned < ang
                    rotation = AngleSensorRoomba(serPort);
                    angTurned = angTurned+abs(rotation);
                    currentRot = currentRot + rotation;
                    [currentPosX, currentPosY, currentRot] =updateCurrent(serPort,currentPosX, currentPosY, currentRot);
                    [tctr, time, orient] = recordOrient(tctr, time, orient, tStart, currentRot);
                    pause(0.1);
               end
            end
        end
        
       [bump, currentPosX, currentPosY, currentRot] = bumpCheckReact(serPort, currentPosX, currentPosY, currentRot);
     

        WallSensor = WallSensorReadRoomba(serPort);
        while(WallSensor == 0 && wallFound && ~bump)

            travelDist(serPort, v, 0.25);
            
             previousPosX = currentPosX;   previousPosY = currentPosY;
           [currentPosX, currentPosY, currentRot] = updateCurrent(serPort,currentPosX, currentPosY, currentRot);
             inStartArea = checkStartArea(currentPosX, currentPosY, previousPosX, previousPosY, wallStartPosX, wallStartPosY, inStartArea, leftStartArea);
            fprintf('2.updateCurrent(serPort,currentPosX: x: %d, y: %d, Rot: %d\n', currentPosX, currentPosY, currentRot);
            
            fprintf('currentPosX: %4.2f, currentPosY: %4.2f, currentRot: %4.2f\n', currentPosX, currentPosY, currentRot);
            figure(posFig); scatter(currentPosX,currentPosY); hold on;
               
            SetFwdVelAngVelCreate(serPort, 0, -0.2);
            
            previousPosX = currentPosX;   previousPosY = currentPosY;
            [currentPosX, currentPosY, currentRot] = updateCurrent(serPort,currentPosX, currentPosY, currentRot);
             inStartArea = checkStartArea(currentPosX, currentPosY, previousPosX, previousPosY, wallStartPosX, wallStartPosY, inStartArea, leftStartArea);
           
             fprintf('currentPosX: %4.2f, currentPosY: %4.2f, currentRot: %4.2f\n', currentPosX, currentPosY, currentRot);
             figure(posFig);
             scatter(currentPosX,currentPosY); hold on;
             
            ang = pi/17;
            angTurned = 0;
            while angTurned < ang
                rotation = AngleSensorRoomba(serPort);
                angTurned = angTurned+abs(rotation);
                currentRot = currentRot + rotation;
                [currentPosX, currentPosY, currentRot] =updateCurrent(serPort,currentPosX, currentPosY, currentRot);
                pause(0.1);
            end
            [currentPosX, currentPosY, currentRot] =updateCurrent(serPort,currentPosX, currentPosY, currentRot);
            
            WallSensor = WallSensorReadRoomba(serPort);
            [bump, currentPosX, currentPosY, currentRot] = bumpCheckReact(serPort, currentPosX, currentPosY, currentRot);
             inStartArea = checkStartArea(currentPosX, currentPosY, previousPosX, previousPosY, wallStartPosX, wallStartPosY, inStartArea, leftStartArea);
        end
        % Briefly pause to avoid continuous loop iteration
        pause(0.1)
    end
    
    figure(posFig);
    title('Robot Position'); xlabel('y position'); ylabel('x position');
    
    figure; 
    plot(time(1:tctr-1), orient(1:tctr-1));
    title('Robot Orientation'); xlabel('time (sec)'); ylabel('Angle (rad)');
    %[time(1:tctr-1) orient(1:tctr-1)]
    
    % Specify output parameter
    finalRad= v/w;
    
    % Stop robot motion
    v= 0;
    w= 0;
    SetFwdVelAngVelCreate(serPort,v,w);
    previousPosX = currentPosX;   previousPosY = currentPosY;
    [currentPosX, currentPosY, currentRot] = updateCurrent(serPort,currentPosX, currentPosY, currentRot);
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
        angTurned= 0;
        while angTurned < ang
               rotation = AngleSensorRoomba(serPort);
               angTurned = angTurned+abs(rotation);
               currentRot = currentRot + rotation;
               [currentPosX, currentPosY, currentRot] =updateCurrent(serPort,currentPosX, currentPosY, currentRot);
               pause(0.1);
        end
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

function [tctr, time, orient] = recordOrient(tctr, time, orient, tStart, currentRot)
    time(tctr) = toc(tStart);
    orient(tctr) = currentRot;
    fprintf('time: %4.2f, rot: %4.2f\n', time(tctr), orient(tctr));
    tctr = tctr + 1;   
end

function inStartArea = checkStartArea(currentPosX, currentPosY, previousPosX, previousPosY, wallStartPosX, wallStartPosY, inStartArea, leftStartArea)
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
% wallStartPosY - Y component of the starting position of the robot after
% it hit a wall
% inStartArea - Boolean marking whether the robot is in the start area.
% leftStartArea-  Boolean marking whether the robot has once left the start
% area.
%
% Output:
% bumped -Boolean marking whether the robot is in the start area.

    enterStartAreaDistThresh = 0.8;

    if ~inStartArea && leftStartArea && pdist([currentPosX, currentPosY; wallStartPosX, wallStartPosY], 'euclidean') < enterStartAreaDistThresh
        inStartArea = 1;
    elseif inStartArea
        if pdist([currentPosX, currentPosY; wallStartPosX, wallStartPosY], 'euclidean') - ...
            pdist([previousPosX, previousPosY; wallStartPosX, wallStartPosY], 'euclidean') > 0
            
            fprintf('hey');
        end
    end
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