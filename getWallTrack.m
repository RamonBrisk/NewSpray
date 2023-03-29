

function [wallSSSpraytrack, wallSLSpraytrack] = getWallTrack(clamp,matchingCircleWall,pc,radius,offsetX,offsetZ,toolDistance,trackWidth)



%计算墙面弧线

matchingCircleWall(:,3) =  matchingCircleWall(:,3) + offsetZ*radius + toolDistance;

pc(3) = pc(3) + offsetZ*radius + toolDistance;

if offsetX < 0
    matchingCircleWall(:,1) = matchingCircleWall(:,1) + offsetX*radius - toolDistance;
    pc(1) = pc(1) + offsetX*radius - toolDistance;
    [row,~] = find(matchingCircleWall(:,1)>pc(1)+40 & matchingCircleWall(:,3)<pc(3));
    arcWall = matchingCircleWall(row,:);
    arcWall = sortrows(arcWall,1,'ascend');
else
    matchingCircleWall(:,1) = matchingCircleWall(:,1) + offsetX*radius + toolDistance;
    pc(1) = pc(1) + offsetX*radius + toolDistance;
    [row,~] = find(matchingCircleWall(:,1)<pc(1)-40 & matchingCircleWall(:,3)<pc(3));
    arcWall = matchingCircleWall(row,:);
    arcWall = sortrows(arcWall,1,'descend');

end


%计算墙面两次喷涂次数
totalNum = ceil((clamp.ymax - clamp.ymin)/trackWidth) + 1;
if mod((clamp.ymax - clamp.ymin), trackWidth)< trackWidth / 2
    totalNum = totalNum - 1;
end
firstNum = floor(totalNum/2);
secondNum = totalNum - firstNum;

wallSSSpraytrack =  calculateWallTrack(clamp,arcWall,pc,firstNum,clamp.ymin,trackWidth,radius);

arcWall(:,2) = arcWall(:,2) +firstNum*trackWidth;
wallSLSpraytrack =  calculateWallTrack(clamp,arcWall,pc,secondNum,clamp.ymin+firstNum*trackWidth,trackWidth,radius);


end
