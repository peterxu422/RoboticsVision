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

    % Set constants for this program
    maxDuration= 1200;  % Max time to allow the program to run (s)
    
    % Initialize loop variables
    tStart= tic;        % Time limit marker
    v= 0.3;               % Forward velocity (m/s)
    w = 0;              %Initialize angular velocity to 0
    
    %Odometry Variables
    currentPosX = 0.0;
    currentPosY = 0.0;
    currentRot = 0.0;
    
    %resetting the Sensors
    dist = DistanceSensorRoomba(serPort);
    ang = AngleSensorRoomba(serPort);
  
        % Select a Mask, from the image
    %clc; clear;
    [img, map1] = imread('ref.jpg');
    D = dir('./imgs/*.jpg');
    %I2 = imcrop(img);
    
    % Create the mask, binary image
    red_mask = img(:,:,1) > 220 & img(:,:,2) < 150 & img(:,:,3) < 150;
    figure(1);
    %imshow(red_mask);
    
    % Calculate centroid and area of blob
    
    %area
    M00 = 0;
    %centroid
    M10 = 0;
    M01 = 0;
    for i= 1:size(red_mask, 1)     %Y Values
        for j= 1:size(red_mask, 2) %X values
           if( red_mask(i,j) == 1)
              % assumes blob colors outside of the blob do not exist 
              M00 = M00 + 1;
              
              M10 = M10 + (j ^ 1) * red_mask(i,j);
              M01 = M01 + (i ^ 1) * red_mask(i,j);
           end
            
        end 
    end
    
    [M00 M10 M01];
    xc = M10/M00;
    yc = M01/M00;
    
    y = 6;
    for x=1:size(D,1)
        
       %Images
       [img, map2] = imread(strcat('./imgs/', D(x).name));       
       red_mask_cur = img(:,:,1) > 220 & img(:,:,2) < 150 & img(:,:,3) < 150;
       iptsetpref('ImshowAxesVisible', 'on'),
       subplot(1,2,1), imshow(red_mask, map1), iptsetpref('ImshowAxesVisible', 'on')
       subplot(1,2,2), imshow(red_mask_cur, map2);
       [M00_cur, xc_cur, yc_cur] = compareImage(red_mask_cur);
       
       fprintf(D(x).name);
       fprintf('\n');
       fprintf('Area - Ref:%d, Cur:%d\n', M00, M00_cur);
       fprintf('Centr - Ref:%3.2f, Cur:%3.2f\n', xc, xc_cur);
       
       ang = pi/10;
       w = 0.2;
       if(xc_cur >= (1.1 * xc))
           %fprintf('Turn right\n');
           SetFwdVelAngVelCreate(serPort, 0, -w);
           angleTurn(serPort, ang, currentPosX, currentPosY, currentRot);
       elseif (xc_cur <= (0.9 * xc))
           %fprintf('Turn left\n');
           SetFwdVelAngVelCreate(serPort, 0, w);
           angleTurn(serPort, ang, currentPosX, currentPosY, currentRot);
       else
           %fprintf('Dont Turn\n');
           SetFwdVelAngVelCreate(serPort, 0, 0);
       end
       
       if(M00_cur >= (1.2 * M00))
           %fprintf('Move backward\n');
           travelDist(serPort, v, -1.0);
       elseif (M00_cur <= (0.8 * M00))
           %fprintf('Move forward\n');
           travelDist(serPort, v, 1.0);
       else
           %fprintf('Dont Move\n');
           SetFwdVelAngVelCreate(serPort, 0, 0);
       end
       
       fprintf('\n');
       
       pause(7);
    end

    
    %Plots the position of the robot. (assumes a start at (0,0) with
    %orientation angle 0.
       %scatter(currentPosX,currentPosY); hold on;
    SetFwdVelAngVelCreate(serPort,v,w);
	
	time = zeros(maxDuration, 1);	% Create arrays for time and orientation to be printed later.
	orient = zeros(maxDuration, 1);
	tctr = 1;
   
    
        
    
    % Specify output parameter
    finalRad= v/w;
    
    % Stop robot motion
    v= 0;
    w= 0;
    SetFwdVelAngVelCreate(serPort,v,w);
    previousPosX = currentPosX;   currentPosY = currentPosY;
    [currentPosX, currentPosY, currentRot] = updateCurrent(serPort,currentPosX, currentPosY, currentRot);
	[tctr, time, orient] = recordOrient(tctr, time, orient, tStart, currentRot); %%
	
	%%

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

function [M00_cur, xc_cur, yc_cur] = compareImage(image)
    M00 = 0;
    M10 = 0;
    M01 = 0;
    for i= 1:size(image, 1)     %Y Values
        for j= 1:size(image, 2) %X values
           if( image(i,j) == 1)
              % assumes blob colors outside of the blob do not exist 
              M00 = M00 + 1;
              
              M10 = M10 + (j ^ 1) * image(i,j);
              M01 = M01 + (i ^ 1) * image(i,j);
           end
            
        end 
    end
    
    M00_cur = M00;
    xc_cur = M10/M00;
    yc_cur = M01/M00;
end