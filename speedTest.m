function  speedTest()
tic();

url = 'http://192.168.1.100:81/snapshot.cgi?user=admin&pwd=&resolution=32&rate=0';

 %fh = image(ss);
 while(1)
     tic()
  img  = getimage(url,'jpg');
  toc()
[door blobSize blobPos] = findDoor(img); 

ant = size(img,2) /2
    for i = 1:size(blobSize,1)
        
            hold on;
        side = sqrt(blobSize(i));
        if(side > 1)
            r = rectangle('Position', [ (blobPos(i,1) /2)  (blobPos(i,2)-side/2)  side side]);
            set(r,'edgecolor','r');
            if door == 1
                set(r,'LineWidth',4);
            end
        end
    end
   fprintf('\nFound Door: %d blobsize: %d x: %f y: %f\n', door, blobSize(1), blobPos(1,1), blobPos(1,2));
    pause(0.2)
 
  %set(fh,'CData',image);
  drawnow;
  pause(0.05);
  toc()
 end
end

 
 function[door blobSize blobPos] =findDoor(img,h,s,v,clamp)
 % Create the mask, binary image
    [min1, max1, min2, max2, min3, max3] = maskThreshHSV(0.6, 0.31, 0.5);
     img = rgb2hsv(img);
    mask = (img(:,:,1) >= min1 & img(:,:,1) <= max1) & (img(:,:,2) >= min2 & img(:,:,2) <= max2);
    pause(0.2);
    imshow([rgb2gray(hsv2rgb(img)) mask]);
    
    [blobSize blobPos] = getBlobs(mask, 1);
    
    if(max(blobSize) > 100000)
        door = 1;
    else
        door = 0;
    end
  
    
 end
 
 function [minr, maxr, ming, maxg, minb, maxb] = maskThreshRGB(r,g,b)
    minr = r* 0.8;
    maxr = r * 1.2;
    ming = g * 0.8;
    maxg = g * 1.2;
    minb = b * 0.8;
    maxb = b * 1.2;
 end
 

function [minh, maxh, mins, maxs, minv, maxv] = maskThreshHSV(h, s, v)
    minh = h * 0.95;   %hue's threshold should be tight
    maxh = h* 1.05;
    mins = s* 0.7;    %saturation's threshold should be more lenient b/c of lighting conditions
    maxs = s* 1.3;
    minv = v* 0.7;
    maxv = v* 1.3;
end
 
function [blobSize blobPos] = getBlobs(threshImg,x)
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
                blobPos(blobInd,1) = blobPos(blobInd,1) + j;
                blobPos(blobInd,2) = blobPos(blobInd,2) + i;
            end
        end
    end
    blobPos(:,1) = blobPos(:,1) ./blobSize;
    blobPos(:,2) = blobPos(:,2) ./blobSize;
    
    largest = max(blobSize);
     n = size(blobSize);
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

