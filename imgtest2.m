function [ output_args ] = imgtest( input_args )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    
    % Select a Mask, from the image
    %clc; clear;
    url = 'http://192.168.1.100:81/snapshot.cgi?user=admin&pwd=&resolution=32&rate=0';
    [img, map1] = imread(url);
    D = dir('./imgs/*.jpg');
    I2 = imcrop(img);
    figure(1);
    imshow(img);
    
%     figure(2);
%     imshow(I2);

    %avgimg = mean(I2);
    %size(avgimg)
    
    
    avgred = mean(I2(:,:,1));
    avggrn = mean(I2(:,:,2));
    avgblu = mean(I2(:,:,3));
    redmask = false;
    grnmask = false;
    blumask = false;
    
    [minr, maxr, ming, maxg, minb, maxb] = maskThresh(I2);
    
    mask = (img(:,:,1) >= minr & img(:,:,1) <= maxr) & (img(:,:,2) >= ming & img(:,:,2) <= maxg) &(img(:,:,3) >= minb & img(:,:,3) <= maxb);
    
    % Create the mask, binary image
    %red_mask = img(:,:,1) > 220 & img(:,:,2) < 150 & img(:,:,3) < 150;
    figure(1);
    imshow(mask);
    
    % Calculate centroid and area of blob
    
    %area
%     M00 = 0;
%     %centroid
%     M10 = 0;
%     M01 = 0;
%     for i= 1:size(mask, 1)     %Y Values
%         for j= 1:size(mask, 2) %X values
%            if( mask(i,j) == 1)
%               % assumes blob colors outside of the blob do not exist 
%               M00 = M00 + 1;
%               
%               M10 = M10 + (j ^ 1) * mask(i,j);
%               M01 = M01 + (i ^ 1) * mask(i,j);
%            end
%             
%         end 
%     end
%     
%     [M00 M10 M01];
%     xc = M10/M00;
%     yc = M01/M00;
%     
%     y = 6;
%     for x=1:size(D,1)
%        [img, map2] = imread(strcat('./imgs2/', D(x).name));       
%        %red_mask_cur = img(:,:,1) > 220 & img(:,:,2) < 150 & img(:,:,3) < 150;
%        imshow(img);
%        if redmask == true
%            mask_cur = img(:,:,1) > 220 & img(:,:,2) < 150 & img(:,:,3) < 150;
%        elseif grnmask == true
%            mask_cur = img(:,:,1) < 150 & img(:,:,2) > 220 & img(:,:,3) < 150;
%        elseif blumask == true
%            mask_cur = img(:,:,1) < 150 & img(:,:,2) < 150 & img(:,:,3) > 220;
%        end
%        
%        iptsetpref('ImshowAxesVisible', 'on'),
%        %subplot(1,2,1), imshow(mask, map1), iptsetpref('ImshowAxesVisible', 'on')
%        %subplot(1,2,2), imshow(mask_cur, map2);
%        [M00_cur, xc_cur, yc_cur] = compareImage(mask_cur);
%        
%        fprintf(D(x).name);
%        fprintf('\n');
%        fprintf('Area - Ref:%d, Cur:%d\n', M00, M00_cur);
%        fprintf('Centr - Ref:%3.2f, Cur:%3.2f\n', xc, xc_cur);
%        if(xc_cur >= (1.1 * xc))
%            fprintf('Turn right\n');
%        elseif (xc_cur <= (0.9 * xc))
%            fprintf('Turn left\n');
%        else
%            fprintf('Dont Turn\n');
%        end
%        
%        if(M00_cur >= (1.2 * M00))
%            fprintf('Move backward\n');
%        elseif (M00_cur <= (0.8 * M00))
%            fprintf('Move forward\n');
%        else
%            fprintf('Dont Move\n');
%        end
%        
%        fprintf('\n');
%        
%        pause(7);
%     end
    
    %[i, dx, dyy] = AreaSelection(img);

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

function [minr, maxr, ming, maxg, minb, maxb] = maskThresh(img)
    minr = mean(mean(img(:,:,1))) * 0.8;
    maxr = mean(mean(img(:,:,1))) * 1.2;
    ming = mean(mean(img(:,:,2))) * 0.8;
    maxg = mean(mean(img(:,:,2))) * 1.2;
    minb = mean(mean(img(:,:,3))) * 0.8;
    maxb = mean(mean(img(:,:,3))) * 1.2;
end

%%%% This function lets tou select desired region on an image%%%%%%%%%%%
%%%% Be sure before you input the image into the function that you read the image using imread%%%%%%%%%%%
function [I2,dataX,dataYY] = AreaSelection(b)

 GH = figure; imshow(b)%%%Shows the input image%%%%
  waitforbuttonpress  %%%%%%Press left click on the image%%%%
  ss = size(b);
  point1 = get(gcf,'CurrentPoint') ;
  rect = [point1(1,1) point1(1,2) 335 219]; %Coordinates of rectangle%
  [r2] = dragrect(rect);                    %%%Drag the rectangle while keeping leftclick pressed and leave the click when region to be selected is decided%%
  [dataX, dataY] = pix2data(r2(1,1),r2(1,2));
  ggr = ss(1,1) - dataY ;
  dataY = ggr+0.5;
  dataX=dataX+0.5;                      %Top left hand side X coordinate of the image%
  dataYY =dataY-r2(1,4);                %Top left hand side Y coordinate of the image%  
  I2 = imcrop(b,[dataX dataYY 335 219]);%Final Cropped image%
  close(GH)
end
