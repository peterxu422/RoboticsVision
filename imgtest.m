function [ output_args ] = imgtest( input_args )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    
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
       if(xc_cur >= (1.1 * xc))
           fprintf('Turn right\n');
       elseif (xc_cur <= (0.9 * xc))
           fprintf('Turn left\n');
       else
           fprintf('Dont Turn\n');
       end
       
       if(M00_cur >= (1.2 * M00))
           fprintf('Move backward\n');
       elseif (M00_cur <= (0.8 * M00))
           fprintf('Move forward\n');
       else
           fprintf('Dont Move\n');
       end
       
       fprintf('\n');
       
       pause(7);
    end
    
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
