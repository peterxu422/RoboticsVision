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
x = 0;
y = 0;
rot = 0;
        % Select a Mask, from the image
    %clc; clear;
    
    % Calculate centroid and area of blob

    enAngle = 0;
    enVal = 0;
    url = 'http://192.168.1.100:81/snapshot.cgi?user=admin&pwd=&resolution=32&rate=0';
    numPoints = zeros(20,1);
    ent = zeros(20,1);
    %The following code can be kind of slow if the camera is being unresponsive
    %(it can be from anywhere between 3fps to 0.3 fps). It is used to determine the
    %best starting direction to find a door.  However, since the robot can
    %be assumed to already be in the right direction, comment out for
    %faster runthrough
    %------------------START COMMENT HERE TO STOP DIRECTION CHECK--------------------
     for i=1 : 20  
         img = getimage(url,'jpg');
         entrImg = img(:,size(img,2)/3:2 * size(img,2)/3,:);
         [door blobSize blobPos] = findDoor(img);
         numPoints(i) = sum(blobSize);
         ent(i) = entropy(rgb2gray(entrImg));
         fprintf('\n Entropy: %d',ent(i));
         SetFwdVelAngVelCreate(serPort, 0, 0.2); 
         [~, ~, ang] = angleTurn(serPort, pi/10,0,0,ang);
        SetFwdVelAngVelCreate(serPort, 0, 0);
         imshow(entrImg);
         pause(0.2);
     end
     highScore = 0;
     highIndex = 1;
     
     %Calculates the score of a direction base on its enthropy value and
     %the number of possible door points found in that direction  The
     %biggest score is memorized and used as the starting direction to find
     %a door.
     for i=1 : size(numPoints,1);  
        score = ent(i) / max(ent)*2 + log(numPoints(i) / max(numPoints));
        fprintf('\ni: %d angle %d score: %f ent: %d numPoints: %d\n', i, i * 18, score,ent(i),log(numPoints(i) / max(numPoints)));
        if score > highScore
            highScore = score;
            highIndex = i;
        end
     end
     
     fprintf('max entropy: %d angle:%d \n', highScore, ( highIndex-1)* pi/10);
 
     %Change the direction of the robot to face in the direction found to
     %have the biggest potential for having doors.
     angDiff = mod((highIndex - 1)*pi/10 + 2* pi - ang,2 * pi)
     if angDiff > pi
         angDiff = (2 * pi - angDiff)  
         SetFwdVelAngVelCreate(serPort, 0, -0.2);
     else    
         SetFwdVelAngVelCreate(serPort, 0, 0.2);
     end
         
     [posX,posY,ang]=angleTurn(serPort,angDiff,0,0,ang);
     SetFwdVelAngVelCreate(serPort, 0, 0);
    %------------------END COMMENT HERE TO STOP DIRECTION CHECK--------------------
    door = 0;
    while( 1 )
        if(door == 0)
            SetFwdVelAngVelCreate(serPort, 0.2, 0.0);
            
            pause(3)
            SetFwdVelAngVelCreate(serPort, 0, 0.0);
            
            %Try to find a door right in front of the robot
            img = getimage(url,'jpg');
            [door blobSize blobPos] = findDoor(img); 
            pause(0.2)
            
            if door == 0
                %Try to find a door to the left of the robot
              SetFwdVelAngVelCreate(serPort, 0, 0.2);
              fprintf('\nTurning left\n');
              [x, y, rot] =  angleTurn(serPort, 1/6 * pi, x, y, rot);

              SetFwdVelAngVelCreate(serPort, 0, 0);
              img = getimage(url,'jpg');
              [door blobSize blobPos] = findDoor(img); 
              
              if door == 0
                  %Try to find a door to the right of the robot
                  fprintf('\nTurning right\n');
                  SetFwdVelAngVelCreate(serPort,0, -0.2);
                  [x, y, rot] =  angleTurn(serPort, 1/3 * pi, x, y, rot);
                  
                  SetFwdVelAngVelCreate(serPort, 0, 0);
                  img = getimage(url,'jpg');
                  [door blobSize blobPos] = findDoor(img); 
                  
                  if door ==0
                      %Try to find a centering back to the middle to
                      %continue down hallway
                       SetFwdVelAngVelCreate(serPort,0, 0.2);
                      [x, y, rot] =  angleTurn(serPort, 1/6 * pi, x, y, rot) ;
                      
                      SetFwdVelAngVelCreate(serPort, 0, 0);
                      img = getimage(url,'jpg');
                      [door blobSize blobPos] = findDoor(img); 
                  end
              end
            end
            
            
 
            %This code was ment to be used to Stay in the middle of the
            %hallway.  However, with the low framerate we were getting, it
            %was impossible to implement a pid controller with it without
            %seriously slowing down the robot (we were getting 1frame/4 sec
            
%             left = entropy(img(:,[1:9 * size(img,2) /20 ],:));
%             right = entropy(img(:,[11 * size(img,2) /20 : end] ,:));
%             if(abs(left - right) > 1)
%                 ang = (left - right)/(left + right / 2) * 0.3;
%             else 
%                 ang = 0
%             end
%             fprintf('\nleft: %f right %f ang %f door:%d numDoors:%d\n', left, right, ang, door, size(blobSize,1));
%             %SetFwdVelAngVelCreate(serPort, 0.2, ang);

        else   
            %This code centers the robot on the door found. The rotational
            %velocity is proportional to the difference in rotation so that
            %if the door is further from the center of the robot, the robot
            %travels more rotational distance to it for each iteration
            while( abs(blobPos(1,2) - 320)/320 > 0.1)
               
                   vel = (320 - blobPos(1,2) )/320;
                   SetFwdVelAngVelCreate(serPort, 0,vel );
                   pause(0.8);
                   SetFwdVelAngVelCreate(serPort, 0,0 );
                   fprintf('\nvel: %f pos:%d mid:%d\n' ,vel,blobPos(1,2),320)
                   
                   img = getimage(url,'jpg');
                   [door blobSize blobPos] = findDoor(img); 
                   
            end
               %Drives at wall until bumped
               SetFwdVelAngVelCreate(serPort, 0,0 );
               fprintf('\n\n Aimed At wall. Going Forward');
               bumped = 0;
               for i = 1: 3
                   %We continue driving until we hit a wall
                   while bumped == 0
                      
                      SetFwdVelAngVelCreate(serPort, 0.2,0 );
                      
                      pause(0.4);
                      [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster ...
                            BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
                      bumped= BumpRight || BumpLeft || BumpFront;
                       %if a wall is hit with the right bumper, we correct
                       %so that the next hit is in the front
                      if BumpRight
                           SetFwdVelAngVelCreate(serPort,0, -0.2);
                          [~, ~, ~] =  angleTurn(serPort, 1/3 * pi, x, y, rot) ;
                           SetFwdVelAngVelCreate(serPort,0, 0);
                      end
                       %if a wall is hit with the left bumper, we correct
                       %so that the next hit is in the front
                       if BumpLeft 
                           SetFwdVelAngVelCreate(serPort,0, 0.2);
                          [~, ~, ~] =  angleTurn(serPort, 1/3 * pi, x, y, rot) ;
                           SetFwdVelAngVelCreate(serPort,0, 0);
                       end
   
                   end
                   
                   BeepRoomba(serPort);
                   fprintf('\n\nKnock\n\n');
                   
                   if i < 3
                        bumped = 0;
                        SetFwdVelAngVelCreate(serPort, -0.2,0 );
                        pause(1);
                   end
                   SetFwdVelAngVelCreate(serPort, 0,0 );
               end
               
            while 1
                fprintf('\n\n DONE RUNNING \n\n');
                   pause(5000);
            end
        end
    end
end
    
    %[blobSize blobPos] = getBlobs(threshImage,0.05);

 function[door blobSize blobPos] =findDoor(img,h,s,v,clamp)
 % This function returns an array of all the likely door blobs, and whether
 % a door is found, in otherwords, whether a blob was found to be big
 % enough to be considered a door
    [min1, max1, min2, max2, min3, max3] = maskThreshHSV(0.6, 0.31, 0.5);
     img = rgb2hsv(img);
    mask = (img(:,:,1) >= min1 & img(:,:,1) <= max1) & (img(:,:,2) >= min2 & img(:,:,2) <= max2);
    
    imshow([rgb2gray(hsv2rgb(img)) mask]);
    [blobSize blobPos] = getBlobs(mask, 0.5);

    fprintf('\bMax Blob: %d Number of blobs: %d\n', max(blobSize), size(blobSize));
    if(max(blobSize) > 65000)
        door = 1;
    else
        door = 0;
    end
    for i = 1: size(blobSize,1)
        side = sqrt(blobSize(i));
        if(side* side > 65000)
            b = rectangle('Position', [ (blobPos(i,2) - side/2)  (blobPos(i,1)-side/2)  side side]);
            set(b, 'edgecolor','b');
            if door == 1
                set(b,'LineWidth',4);
            end
        end
    end
    
 end
 
 function [minr, maxr, ming, maxg, minb, maxb] = maskThreshRGB(r,g,b)
 %generates mins and maxes for an RGB threshold
    minr = r* 0.8;
    maxr = r * 1.2;
    ming = g * 0.8;
    maxg = g * 1.2;
    minb = b * 0.8;
    maxb = b * 1.2;
 end
 

function [minh, maxh, mins, maxs, minv, maxv] = maskThreshHSV(h, s, v)
%Generates the mins and maxes for a HSV threshold
    minh = h * 0.95;   %hue's threshold should be tight
    maxh = h* 1.05;
    mins = s* 0.7;    %saturation's threshold should be more lenient b/c of lighting conditions
    maxs = s* 1.3;
    minv = v* 0.7;
    maxv = v* 1.3;
end
 
function [blobSize blobPos] = getBlobs(threshImg,minSizeRatio)
%Generates all the blobs from the threshold image and returns them as an
%array of blob sizes and an array of blob positions(x,y).  The minSizeRatio input
%parameter allows for only blobs of size at least minSizeRatio*the biggest
%blob to be returned.  All other blobs are eliminated from the list.
count = 1;
matchingColors = [-1 -1];
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
                blobInd = threshImg(i,j);
                 
                blobSize(blobInd,1) = blobSize(blobInd,1) + 1; 
                blobPos(blobInd,1) = blobPos(blobInd,1) + i;
                blobPos(blobInd,2) = blobPos(blobInd,2) + j;
            end
        end
    end
    blobPos(:,1) = blobPos(:,1) ./blobSize;
    blobPos(:,2) = blobPos(:,2) ./blobSize;
    
    largest = max(blobSize);
     n = size(blobSize);
     i = 1;
    while i <= n 
        if blobSize(i) < largest * minSizeRatio
            blobSize = blobSize([1:i-1, i+1:end]);
            blobPos = blobPos([1:i-1, i+1:end],:);
            n = n -1;
            i = i - 1;
        end
        i = i + 1;
    end
    
end


function matchingColors = addMatch(matchingColors,A,B)
%This function adds a match to the match table.  The match table is a list
%of all blobs values that are found to actually be the same blob.  This is
%later used to unify blobs into bigger blobs if they are adjacent
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
%Turns ang degrees in the direction dictated by the angular speed.  This
%function returns a posX posY rot odometry as it was used in previous
%homeworks, however these are not used in this submission.
    angTurned = 0;
    fprintf('\nAngle turning: %f Goal: %f \n', angTurned, ang);
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
