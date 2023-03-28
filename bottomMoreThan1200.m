function bottomMoreThan1200(clamp, radius, deltaZ)

close all;
debug = 1;
toolDistance = 190;
trackWidth = 190;

arcStartMark = [-80000, -80000, -80000, -80000, -80000, -80000];
arcEndMark = [-150000, -150000, -150000, -150000, -150000, -150000];



h = clamp.panel - deltaZ; % 高度
r = radius;
pc = [(clamp.xmin + clamp.xmax)/2, clamp.ymin, h];
pos = [pc(1), pc(2)]; % 圆心位置
t=0:0.1:(2*pi);  % 圆滑性设置
t=[t,0];
matchingCircle = [(pos(1)+r*sin(t))', (pos(2)+ones(size(t)))',(h+r*cos(t))'];




[row, ~]= find(matchingCircle(:,3) > clamp.zmin - 30);
headSprayTrack = matchingCircle(row,:);
headSprayTrack = sortrows(headSprayTrack,1,'ascend');


headSprayTrack = [headSprayTrack(1,:); headSprayTrack(floor(size(headSprayTrack,1)/2),:); headSprayTrack(end,:)];
headSprayTrack(:,2) = clamp.ymin - toolDistance;
yDirectionABC = [176,0,40];
headSprayTrack = [headSprayTrack,repmat(yDirectionABC,size(headSprayTrack,1),1)];
headSprayTrack = [headSprayTrack(1,:);arcStartMark;headSprayTrack;arcEndMark];




endSprayTrack = matchingCircle(row,:);
endSprayTrack = sortrows(endSprayTrack,1,'ascend');
endSprayTrack = [endSprayTrack(1,:); endSprayTrack(floor(size(endSprayTrack,1)/2),:); endSprayTrack(end,:)];
endSprayTrack(:,2) = clamp.ymax + toolDistance;
yDirectionABC = [176,0,40];
endSprayTrack = [endSprayTrack,repmat(yDirectionABC,size(endSprayTrack,1),1)];
endSprayTrack = [endSprayTrack(1,:);arcStartMark;endSprayTrack;arcEndMark];




matchingCircleWall = matchingCircle;
matchingCircleWall(:,1) = matchingCircleWall(:,1) - 1.1*radius - toolDistance;
matchingCircleWall(:,3) =  matchingCircleWall(:,3) + 1.8*radius + toolDistance;

pc(1) = pc(1) - 1.1*radius - toolDistance;

pc(3) = pc(3) + 1.8*radius + toolDistance;

[row,~] = find(matchingCircleWall(:,1)>pc(1) & matchingCircleWall(:,3)<pc(3));

arcWall = matchingCircleWall(row,:);























if debug == 1

pcshow(clamp.pointCloud);
hold on
pcshow(pointCloud(matchingCircle,"Color",'cyan'));
restoredHeadSprayTrack = restoreTrack(headSprayTrack);
restoredEndSprayTrack = restoreTrack(endSprayTrack);
hold on 
plot3(restoredHeadSprayTrack(:,1), restoredHeadSprayTrack(:,2), restoredHeadSprayTrack(:,3),'.-blue')
hold on 
plot3(restoredEndSprayTrack(:,1), restoredEndSprayTrack(:,2), restoredEndSprayTrack(:,3),'.-blue')

% hold on
% pcshow(pointCloud(matchingCircleWall,"Color",'cyan'));

hold on
pcshow(pointCloud(arcWall,"Color",'cyan'));

end












end