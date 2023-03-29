

function wallSSSpraytrack =  calculateWallTrack(clamp,arcWall,pc,time,startValue,trackWidth,radius)

arcStartMark = [-80000, -80000, -80000, -80000, -80000, -80000];
arcEndMark = [-150000, -150000, -150000, -150000, -150000, -150000];
%计算每次喷涂高度
summitList = findSummit(clamp,"y",time,trackWidth,startValue);
allTracksDiscrete = zeros(6,6,time);
yCurrent = startValue;
xDirection = [90,0,90];
for i=1:time
    allTracksDiscrete(2,:,i) = [arcWall(1,:),90, rad2deg(acos((arcWall(1,1) - pc(1))/radius)), 90];
    allTracksDiscrete(3,:,i) = [arcWall(floor(size(arcWall,1)/2),:),90, rad2deg(acos((arcWall(floor(size(arcWall,1)/2),1) - pc(1))/radius)), 90];
    allTracksDiscrete(4,:,i) = [arcWall(end,:),90, rad2deg(acos((arcWall(end,1) - pc(1))/radius)), 90];
    allTracksDiscrete(6,:,i) = allTracksDiscrete(4,:,i);
    allTracksDiscrete(2:4,2,i) = yCurrent;
    allTracksDiscrete(6,2,i) = yCurrent;
    allTracksDiscrete(1,:,i) = arcStartMark;
    allTracksDiscrete(5,:,i) = arcEndMark;

    if arcWall(end,3)<summitList(i)
        allTracksDiscrete(6,3:6,i) = [summitList(i),xDirection];
    end
    yCurrent = yCurrent + trackWidth;
end
lines = 0;
% 遍历每一页，计算行数和
for i = 1:size(allTracksDiscrete, 3) % 循环遍历每一页
    num_rows = size(allTracksDiscrete(:,:,i), 1); % 获取该页的行数
    lines = lines + num_rows;
end
wallSSSpraytrack = zeros(lines,6);
timeMarker = 1;
for i=1:time
    %第一个零行
    idx = find(all(wallSSSpraytrack == 0, 2), 1);
    if timeMarker > 0
        wallSSSpraytrack(idx:idx+size(allTracksDiscrete(:,:,i),1)-1,:) =  allTracksDiscrete(:,:,i);
    else
        wallSSSpraytrack(idx:idx+size(allTracksDiscrete(:,:,i),1)-1,:) =  reverseTrack(allTracksDiscrete(:,:,i));
    end
    timeMarker = -1 * timeMarker;
end
end