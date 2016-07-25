%Grant Tinkham
%finds the full red rectangle in the image_input

function [ xCentroid, yCentroid, image_input ] = find_target_centroid( image_input )

[x,y] = find_target_corner(image_input); % call the find_target_corner function to find corner

x0 = x;

y0 = y;

while 1 == is_red(image_input, y, x+1) % Top right is found using this function
   
    x = x+1; 
end

while 1 == is_red(image_input, y+1, x)% Bottom Right is gound using this function
   
    y = y+1;    
end

xCentroid = (x0 + x)/2;

yCentroid = (y0 + y)/2;



        for i = x0:x
          for j = y0:y 
             image_input(j,i,1) = 0;
             image_input(j,i,2) = 0;
             image_input(j,i,3) = 0;    

          end
        end
end