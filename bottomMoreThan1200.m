function bottomMoreThan1200(clamp, radius, deltaZ)

debug = 1;
toolDistance = 190;
trackWidth = 190;

arcStartMark = [-80000, -80000, -80000, -80000, -80000, -80000];
arcEndMark = [-150000, -150000, -150000, -150000, -150000, -150000];



pcshow(clamp.pointCloud);

h = clamp.panel - deltaZ; % 高度
r = radius;
pos = [(clamp.xmin + clamp.xmax)/2, clamp.ymin]; % 圆心位置
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










if debug == 1

hold on
pcshow(pointCloud(matchingCircle,"Color",'cyan'));
end

restoredHeadSprayTrack = restoreTrack(headSprayTrack);
restoredEndSprayTrack = restoreTrack(endSprayTrack);
hold on 
plot3(restoredHeadSprayTrack(:,1), restoredHeadSprayTrack(:,2), restoredHeadSprayTrack(:,3),'.-blue')
hold on 
plot3(restoredEndSprayTrack(:,1), restoredEndSprayTrack(:,2), restoredEndSprayTrack(:,3),'.-blue')












end