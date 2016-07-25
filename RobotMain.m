%%%%%%%%%%%%
%Grant Tinkham
% 4/10/14
%%%%%%%%%%%%%%%%%%%%%

%This file takes a .bmp file and translates it into 
6 coordinates, that are then sent to the Arduino Uno
clc, clear

image_feed = imread('TargetPlateP1SideB2.bmp');  %  Loads image containing target locations

[xC,yC,newImage_feed] = find_all_target_centroids(image_feed); % calls find_all_target_centroids and finds all centroids. Proceeds to black them out 

hold on
image(newImage_feed) % Displays image
axis image 
plot(xC,yC,'g x') % Overlays target centorids in red

Arduino = serial('COM4', 'BaudRate', 9600)

fopen(Arduino);

X_highbyte = floor(xC ./ 256);
X_lowbyte = xC - 256 .* X_highbyte;


Y_highbyte = floor(yC ./ 256);
Y_lowbyte = yC - 256 .* Y_highbyte;


for i = 1:6
    fwrite(Arduino, X_highbyte(i));
    fwrite(Arduino, X_lowbyte(i));
    while(Arduino.BytesAvailable == 0)
    end 
    
   message = fscanf(Arduino);
   disp(message); 
   
    
end

for i = 1:6
    fwrite(Arduino, Y_highbyte(i));
    fwrite(Arduino, Y_lowbyte(i));
    while(Arduino.BytesAvailable == 0)
    end 
    
   message = fscanf(Arduino);
   disp(message); 
   
    
end


for i = 1:3600
   
    message = fscanf(Arduino);
   
   disp(message); 
   
    
end
fclose(Arduino);