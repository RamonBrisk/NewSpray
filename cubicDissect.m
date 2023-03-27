    
function originalPoints = cubicDissect(originalPoints,xmin,xmax,ymin,ymax,zmin,zmax)

    %x方向滤波
    [row,~] = find(originalPoints(:,1)<xmin);
    originalPoints(row,:) = [];
    [row,~] = find(originalPoints(:,1)>xmax);
    originalPoints(row,:) = [];
    
    %y方向滤波
    [row,~] = find(originalPoints(:,2)<ymin);
    originalPoints(row,:) = [];
    [row,~] = find(originalPoints(:,2)>ymax);
    originalPoints(row,:) = [];

    %z方向滤波
    [row,~] = find(originalPoints(:,3)<zmin);
    originalPoints(row,:) = [];
    [row,~] = find(originalPoints(:,3)>zmax);
    originalPoints(row,:) = [];

    
end

