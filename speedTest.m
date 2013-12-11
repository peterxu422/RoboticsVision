function  speedTest()
tic();

url = 'http://192.168.1.103:81/snapshot.cgi?user=admin&pwd=&resolution=32&rate=0';

 %fh = image(ss);
 while(1)
     tic()
  image  = getimage(url,'jpg');
  imshow(image);
  %set(fh,'CData',image);
  drawnow;
  pause(0.05);
  toc()
end