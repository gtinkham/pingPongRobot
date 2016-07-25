%%%%%%%%%%%%
%Grant Tinkham
% inputs - Picture array, Y coordinates first, X Coordinates second
% outputs - 1 for True, 0 for False
%%%%%%%%%%%%%%%%%%%%%

function [output] = is_red(picture, Y, X)
%The purpose of this function is to determine whether a point on a
%given picture array is the value of the color red (255,0,0)


if picture(Y,X,1)==255 &&  picture(Y,X,2)==0 && picture(Y,X,3) ==0 
    
    output= 1; %this is the condition if true, just meant to return the nuumber 1
else
    output= 0; %condition if false, returns the number 0

end

end
