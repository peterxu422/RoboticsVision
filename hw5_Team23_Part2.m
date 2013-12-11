% HW1- Team 23
%Frederik Clinckemaillie - fec2109
%Peter Xu - px2117

function hw5_Team23_Part2(serPort)
% Simple program for the iRobot Create or the associated simulator.  This
% program will make the robot go straight until a wall is reached, and then
% follow that wall until it arrives within a certain threshold of its
% original value.

% Input:
% serPort - Serial port object, used for communicating over bluetooth
dist = DistanceSensorRoomba(serPort);
ang = AngleSensorRoomba(serPort);
ang= 0;
v = 0.2;
w = 0.2;
        % Select a Mask, from the image
    %clc; clear;
    
    % Calculate centroid and area of blob
    matchingColors = [-1 -1];
    enAngle = 0;
    enVal = 0;
    url = 'http://192.168.1.103:81/snapshot.cgi?user=admin&pwd=&resolution=32&rate=0';

    
    for i=1 : 12  
        img = getimage(url,'jpg');
        entrImg = img(:,size(img,2)/3:2 * size(img,2)/3,:);
        ent = entropy(rgb2gray(entrImg));
        ang;
        fprintf('entropy: %d angle:%d n', ent, ang);
        if(ent > enVal)
            enVal = ent;
            enAngle = ang;
        end
        
    SetFwdVelAngVelCreate(serPort, 0, 0.2); 
        [posX, posY, ang] = angleTurn(serPort,15 * pi/180,0,0,ang);
        imshow(entrImg);
        pause(3);
        
    SetFwdVelAngVelCreate(serPort, 0, 0);
    end
            fprintf('max entropy: %d angle:%d n', enVal, enAngle);

    angDiff = mod(enAngle + 2* pi - ang,2 * pi)
    
    if angDiff > pi
        angDiff = (2 * pi - angDiff)  
        SetFwdVelAngVelCreate(serPort, 0, -0.2);
    else    
        SetFwdVelAngVelCreate(serPort, 0, 0.2);
    end
        
    [posX,posY,ang]=angleTurn(serPort,angDiff,0,0,ang);
    
    SetFwdVelAngVelCreate(serPort, 0.2, 0);
    
    while( 1 )
    
        door = findDoor(img);
        
        if(door == 0)
            left = entropy(img(:,0:size(img,2) /2 ,:));
            right = entropy(img(:,size(img,2) /2 : end ,:));
            ang = (left - right)/(left + right / 2) * 0.3;
            SetFwdVelAngVelCreate(serPort, 0.2, ang);
        else
        end
    end
    
    %[blobSize blobPos] = getBlobs(threshImage,0.05);

  end    

 function[doorsize doorPos] =findDoor(img,h,s,v,clamp)
 
 end
  
 
function [blobSize blobPos] = getBlobs(threshImg,size)
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
    
    size(matchingColors)
    matchingColors =  sortrows(matchingColors,[-1 2])
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
    
    largest = max(blobSize)
     n = size(blobSize,1);
     i = 1;
    while i <= n 
        if blobSize(i) <= largest * size
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
        %fprintf('Angle turning: %f Goal: %f rotation %f \n', angTurned, ang,rotation);
        rot = rot + rotation;
        [posX, posY, rot] =updateCurrent(serPort,posX, posY, rot);
        pause(0.1);
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
