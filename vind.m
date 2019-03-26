function [array] = vind(x,y)
array=[];
for i=1:length(x)
    if isequal(x(i),y)
        array=[array;i];
    end
end
end

