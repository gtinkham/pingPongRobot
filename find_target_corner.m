%%%%%%%%%%%%
%Grant Tinkham
% inputs - Picture array
% outputs - [X, Y] coordinates
%%%%%%%%%%%%%%%%%%%%%

function [X,Y] = find_target_corner(picture)
%the purpose of this function is to utilize the "is_red" function to
%determine the first red corner of a red target block, This is accomplished
%by using two for loops and going through the picture array rows, and
%columns 


[rows,cols,colors] = size(picture); 

for row = 1:rows
  
    for col = 1:cols
     
        output = is_red(picture, row, col); 
    
        if output == 1 ; 
            X=col
            Y=row
     
            return %this ends the loop as soon as the first completely red pixel is reached
            
        end
        
    end
    
end

end



        
        
        
        