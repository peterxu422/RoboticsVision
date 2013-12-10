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
    d = 0.3;
    %Odometry Variables
    currentPosX = 0.0;
    currentPosY = 0.0;
    currentRot = 0.0;
    
    %resetting the Sensors
    dist = DistanceSensorRoomba(serPort);
    ang = AngleSensorRoomba(serPort);
    
        % Select a Mask, from the image
    %clc; clear;
    url = 'http://192.168.1.100:81/snapshot.cgi?user=admin&pwd=&resolution=32&rate=0';
    [img, map1] = imread(url);
    figure(1);
    imshow(img);
    %D = dir('./imgs/*.jpg');
    I2 = imcrop(img);
    img = rgb2hsv(img);
    
    % Create the mask, binary image
    [min1, max1, min2, max2, min3, max3] = maskThreshHSV(I2);
    
    mask = (img(:,:,1) >= min1 & img(:,:,1) <= max1) & (img(:,:,2) >= min2 & img(:,:,2) <= max2) & (img(:,:,3) >= min3 & img(:,:,3) <= max3);
    figure(1);
    imshow(mask);
    % Calculate centroid and area of blob
    
    %area
    M00 = 0;
    %centroid
    M10 = 0;
    M01 = 0;
    for i= 1:size(mask, 1)     %Y Values
        for j= 1:size(mask, 2) %X values
           if( mask(i,j) == 1)
              % assumes blob colors outside of the blob do not exist 
              M00 = M00 + 1;
              
              M10 = M10 + (j ^ 1) * mask(i,j);
              M01 = M01 + (i ^ 1) * mask(i,j);
           end
            
        end 
    end
    
    [M00 M10 M01];
    xc = M10/M00;
    yc = M01/M00;
    
    while true
        
       %Images
       [img, map2] = imread(url);   
       img = rgb2hsv(img);
       mask_cur = (img(:,:,1) >= min1 & img(:,:,1) <= max1) & (img(:,:,2) >= min2 & img(:,:,2) <= max2) & (img(:,:,3) >= min3 & img(:,:,3) <= max3);
       figure(1);
       iptsetpref('ImshowAxesVisible', 'on'),
       subplot(1,2,1), imshow(mask, map1), iptsetpref('ImshowAxesVisible', 'on')
       subplot(1,2,2), imshow(mask_cur, map2);
       [M00_cur, xc_cur, yc_cur] = compareImage(mask_cur);
       
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
           travelDist(serPort, v, -d);
       elseif (M00_cur <= (0.8 * M00))
           %fprintf('Move forward\n');
           travelDist(serPort, v, d);
       else
           %fprintf('Dont Move\n');
           SetFwdVelAngVelCreate(serPort, 0, 0);
       end
       
       fprintf('\n');
       
       pause(2.7);
    end

    
    %Plots the position of the robot. (assumes a start at (0,0) with
    %orientation angle 0.
       %scatter(currentPosX,currentPosY); hold on;
    SetFwdVelAngVelCreate(serPort,v,w);
        
    
    % Specify output parameter
    finalRad= v/w;
    
    % Stop robot motion
    v= 0;
    w= 0;
    SetFwdVelAngVelCreate(serPort,v,w);

	
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

function [minr, maxr, ming, maxg, minb, maxb] = maskThreshRGB(img)
    minr = mean(mean(img(:,:,1))) * 0.8;
    maxr = mean(mean(img(:,:,1))) * 1.2;
    ming = mean(mean(img(:,:,2))) * 0.8;
    maxg = mean(mean(img(:,:,2))) * 1.2;
    minb = mean(mean(img(:,:,3))) * 0.8;
    maxb = mean(mean(img(:,:,3))) * 1.2;
end

function [minh, maxh, mins, maxs, minv, maxv] = maskThreshHSV(img)
    img = rgb2hsv(img);
    %assume already in hsv
    minh = mean(mean(img(:,:,1))) * 0.95;   %hue's threshold should be tight
    maxh = mean(mean(img(:,:,1))) * 1.05;
    mins = mean(mean(img(:,:,2))) * 0.7;    %saturation's threshold should be more lenient b/c of lighting conditions
    maxs = mean(mean(img(:,:,2))) * 1.3;
    minv = mean(mean(img(:,:,3))) * 0.7;
    maxv = mean(mean(img(:,:,3))) * 1.3;
end