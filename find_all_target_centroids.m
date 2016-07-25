%Grant Tinkham
%
%This function finds the center of all 6 targets


function [ xCentroid, yCentroid, image_input ] = find_all_target_centroids( image_input )

for i = 1:6
    [x,y,image_input] = find_target_centroid(image_input); % Calls the find_target centroid function to find centroids of targets
    
    xCentroid(i) = x;
   
    yCentroid(i) = y;
%     saves the centroid to the current location of in the for loop
%     specified by i

    imshow(image_input)

end
end