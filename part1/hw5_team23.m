% HW1- Team 23
%Frederik Clinckemaillie - fec2109
%Peter Xu - px2117

function threshImg = hw5_Team23(serPort)
% Simple program for the iRobot Create or the associated simulator.  This
% program will make the robot go straight until a wall is reached, and then
% follow that wall until it arrives within a certain threshold of its
% original value.

% Input:
% serPort - Serial port object, used for communicating over bluetooth

    % Set constants for this program
    % Max time to allow the program to run (s)
    
    % Initialize loop variables
    % Time limit marker
    v= 0.3;               % Forward velocity (m/s)
    w = 0;              %Initialize angular velocity to 0
    d = 0.3;
    
%     
%      SetFwdVelAngVelCreate(serPort, 0, 0);
%      travelDist(serPort, v, 0);
%      pause(10);
    
    %Odometry Variables
    currentPosX = 0.0;
    currentPosY = 0.0;
    currentRot = 0.0;

    %resetting the Sensors
    dist = DistanceSensorRoomba(serPort);
    ang = AngleSensorRoomba(serPort);

     
%     SetFwdVelAngVelCreate(serPort, 0, 0.2);
%     deg = 5;
%     ang = deg * pi/180;
%     angleTurn(serPort, ang, currentPosX, currentPosY, currentRot);
%     SetFwdVelAngVelCreate(serPort, 0, 0);
%     pause(20);
    
        % Select a Mask, from the image
    %clc; clear;
    url = 'http://192.168.1.101:81/snapshot.cgi?user=admin&pwd=&resolution=32&rate=0'; 
    img = getimage(url, 'jpg');
    figure(1);
    imshow(img);
    I2 = imcrop(img);
   
    
    % Create the mask, binary image
    [min1, max1, min2, max2, min3, max3] = maskThreshHSV(I2);
    
    
    %D = dir('./imgs/*.jpg');
    %I2 = imcrop(img);
    
    % Create the mask, binary image

    img = rgb2hsv(img);
    mask = (img(:,:,1) >= min1 & img(:,:,1) <= max1) & (img(:,:,2) >= min2 & img(:,:,2) <= max2) & (img(:,:,3) >= min3 & img(:,:,3) <= max3);
%     figure(1);
%     imshow(mask);
    % Calculate centroid and area of blob
    matchingColors = [-1 -1];
    %size(threshImg)
    %count = 1;
    [blobSize blobPos] = getBlobs(mask,1);
   
    while true
        
       %Images
       img = getimage(url, 'jpg');   
       img = rgb2hsv(img);
       mask_cur = (img(:,:,1) >= min1 & img(:,:,1) <= max1) & (img(:,:,2) >= min2 & img(:,:,2) <= max2) & (img(:,:,3) >= min3 & img(:,:,3) <= max3);
%        figure(1);
%        iptsetpref('ImshowAxesVisible', 'on'),
%        subplot(1,2,1), imshow(mask, map1), iptsetpref('ImshowAxesVisible', 'on')
%        subplot(1,2,2), imshow(mask_cur, map2);
       %[M00_cur, xc_cur, yc_cur] = compareImage(mask_cur);
       %figure(1);
       %imshow(mask_cur);
       [blobSzCur blobPosCur] = getBlobs(mask_cur, 1);
       
       fprintf('\n');
       %fprintf('Area - Ref:%d, Cur:%d\n', blobSize, blobSzCur);
       %fprintf('Centr - Ref:%3.2f, Cur:%3.2f\n', blobPos(2), blobPosCur(2));
       fprintf('Centr x: %3.2f, y:%3.2f\n', blobPosCur(2), blobPosCur(1));
       
       centrdiff = blobPosCur(2) - 320;
       ang = abs(centrdiff/320) * 20;
       w = 0.2;
       
       if (centrdiff <= -50)
          fprintf('Turn left %3.2f degrees\n', ang);
          ang = ang * pi/180;
          SetFwdVelAngVelCreate(serPort, 0, w);
          angleTurn(serPort, ang, currentPosX, currentPosY, currentRot);
          SetFwdVelAngVelCreate(serPort, 0, 0);
       elseif (centrdiff >= 50)
          fprintf('Turn right %3.2f degrees\n', ang);
          ang = ang * pi/180;
          SetFwdVelAngVelCreate(serPort, 0, -w);
          angleTurn(serPort, ang, currentPosX, currentPosY, currentRot);
          SetFwdVelAngVelCreate(serPort, 0, 0);
       else
           fprintf('Dont Turn\n');
           SetFwdVelAngVelCreate(serPort, 0, 0);
           %pause(5);
           if (blobSzCur > (1.2 * blobSize))
              %Move back
              %k = blobSzCur/blobSize;
              dist = (blobSzCur/blobSize) * (0.1);
              travelDist(serPort, v, -dist);
           elseif (blobSzCur < (0.8 * blobSize))
              %Move forward
              if (blobSzCur > 3000)
                 % k = blobSize/blobSzCur;
                 dist = (blobSize/blobSzCur) * (0.2);
              else
                  dist = 0.2; 
              end
              travelDist(serPort, v, dist);
           else
               SetFwdVelAngVelCreate(serPort, 0, 0);
           end
           
       end
       
       
%        x = ceil(blobPosCur(2));
%        y = ceil(blobPosCur(1));
%        
%        if(x > 0 && y > 0)
%            dimg = mask_cur;
%            fprintf('drawn loc: %d, %d', x, y);
%            dimg([y:(y+20)],[x:(x+20)]) = 0;
%            imshow(dimg);   
%        end
       
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

end    


function [blobSize blobPos] = getBlobs(threshImg,x)
    matchingColors = [-1 -1];
	count = 1;
    for i = 1 : size(threshImg,1)
        for j = 1 : size(threshImg,2)
            if threshImg(i,j) > 0 
                if i > 1
                    L = threshImg(i-1,j);
                else
                    L = 0;
                end
                if j > 1
                U = threshImg(i,j-1);
                else
                    U = 0;
                end

                if L > 0 && U == 0
                    threshImg(i,j) = L;
                elseif U > 0 && L == 0 
                    threshImg(i,j) = U;
                elseif L > 0 && U > 0
                    threshImg(i,j) = L;
                    temp = [L U];
                    
                    matchingColors = addMatch(matchingColors,L,U);
                end
                
                if L == 0 && U == 0    
                    threshImg(i,j) = count;
                    count = count + 1;
                end
            end
        end
    end
    
    matchingColors =  sortrows(matchingColors,[-1 2]);
    blobSize = zeros(count,1);
    blobPos = zeros(count,2);
    %
    for i = 1 : size(threshImg,1)
        for j = 1 : size(threshImg,2)
            if threshImg(i,j) > 0
                for k = 1: size(matchingColors,1)
                    if matchingColors(k,1) == threshImg(i,j) && matchingColors(k,1) > matchingColors(k,2) 
                        threshImg(i,j) = matchingColors(k,2);
                        k =0;
                    end
                end
                blobSize(threshImg(i,j)) = blobSize(threshImg(i,j)) + 1; 
                blobPos(threshImg(i,j),1) = blobPos(threshImg(i,j),1) + i;
                blobPos(threshImg(i,j),2) = blobPos(threshImg(i,j),2) + j;
            end
        end
    end
    blobPos(:,1) = blobPos(:,1) ./blobSize;
    blobPos(:,2) = blobPos(:,2) ./blobSize;
    
    largest = max(blobSize);
     n = size(blobSize,1);
     i = 1;
    while i <= n 
        if blobSize(i) < largest * x
            blobSize = blobSize([1:i-1, i+1:end]);
            blobPos = blobPos([1:i-1, i+1:end],:);
            n = n -1;
            i = i - 1;
        end
        i = i + 1;
    end
    
end


function matchingColors = addMatch(matchingColors,A,B)
if A ~= B
    temp = [A B];
    [~,indx]=ismember(temp,matchingColors,'rows');

    if(indx < 1 && B ~= A)
        matchingColors = [matchingColors;temp]; 
        matchingColors = [matchingColors; [B A]]; 
    end
end
end

% function [bumped, currentPosX, currentPosY, currentRot] = bumpCheckReact(serPort, currentPosX, currentPosY, currentRot)
% % Check bump sensors and steer the robot away from obstacles if necessary
% %
% % Input:
% % serPort - Serial port object, used for communicating over bluetooth
% % currentPosX - X component of the current position of the robot
% % currentPosY - Y component of the current position of the robot
% % currentRot - Current rotation of the robot
% %
% % Output:
% % bumped - Boolean, true if bump sensor is activated
% % currentPosX - Updated X component of the current position of the robot
% % currentPosY - Updated Y component of the current position of the robot
% % currentRot - Updated current rotation of the robot
% 
%     % Check bump sensors (ignore wheel drop sensors)
%     [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster ...
%         BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
%     bumped= BumpRight || BumpLeft || BumpFront;
%     
%     % Halt forward motion and turn only if bumped
%     if bumped
%         %AngleSensorRoomba(serPort);     % Reset angular odometry
%         %v= 0.45;       % Forward velocity
%         %w= v2w(v);  % Angular velocity
%         v = 0;
%         SetFwdVelAngVelCreate(serPort, v, 0);
%         
%         % Turn away from obstacle
%         ang = 0;
%         if BumpFront
%             w = 0.2;
%             WallSensor = WallSensorReadRoomba(serPort);
%             %If WallSensor == 1 and BumpFront, then it's in a corner and
%             %must turn CC pi/2
%             if WallSensor
%                 SetFwdVelAngVelCreate(serPort, v, w);
%                 ang=pi/2;
%             else
%                 %w = v2w(v);
%                 SetFwdVelAngVelCreate(serPort,v,w);  % Turn counter-clockwise
%                 ang= pi/2;                          % Turn further
%             end
%         elseif BumpLeft
%             w = 0.2;
%            SetFwdVelAngVelCreate(serPort,v,w); % Turn clockwise
%             ang= 3*pi/4;
%         elseif BumpRight
%             w = 0.2;
%            SetFwdVelAngVelCreate(serPort,v,w);  % Turn counter-clockwise
%             ang= pi/19;  % Angle to turn
%         end
%         
%         [currentPosX, currentPosY, currentRot] = updateCurrent(serPort,currentPosX, currentPosY, currentRot);        
%         
%         % Wait for turn to complete 
%         [currentPosX, currentPosY, currentRot] =angleTurn(serPort, ang, currentPosX, currentPosY, currentRot);
%     end
% end

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

% function [M00_cur, xc_cur, yc_cur] = compareImage(image)
%     M00 = 0;
%     M10 = 0;
%     M01 = 0;
%     for i= 1:size(image, 1)     %Y Values
%         for j= 1:size(image, 2) %X values
%            if( image(i,j) == 1)
%               % assumes blob colors outside of the blob do not exist 
%               M00 = M00 + 1;
%               
%               M10 = M10 + (j ^ 1) * image(i,j);
%               M01 = M01 + (i ^ 1) * image(i,j);
%            end
%             
%         end 
%     end
%     
%     M00_cur = M00;
%     xc_cur = M10/M00;
%     yc_cur = M01/M00;
% end

% function [minr, maxr, ming, maxg, minb, maxb] = maskThreshRGB(img)
%     minr = mean(mean(img(:,:,1))) * 0.8;
%     maxr = mean(mean(img(:,:,1))) * 1.2;
%     ming = mean(mean(img(:,:,2))) * 0.8;
%     maxg = mean(mean(img(:,:,2))) * 1.2;
%     minb = mean(mean(img(:,:,3))) * 0.8;
%     maxb = mean(mean(img(:,:,3))) * 1.2;
% end

function [minh, maxh, mins, maxs, minv, maxv] = maskThreshHSV(img)
    img = rgb2hsv(img);
    meanh = mean(mean(img(:,:,1)));
    means = mean(mean(img(:,:,2)));
    meanv = mean(mean(img(:,:,3)));
    
    %fprintf('HSV VALUES %3.2f, %3.2f, %3.2f\n', meanh, means, meanv);
    
    minh = meanh * 0.95;   %hue's threshold should be tight
    maxh = meanh * 1.05;
    mins = means * 0.7;    %saturation's threshold should be more lenient b/c of lighting conditions
    maxs = means * 1.3;
    minv = meanv * 0.7;
    maxv = meanv * 1.3;
end