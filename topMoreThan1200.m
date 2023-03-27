function topMoreThan1200(clamp)

twoSideDistance = 150;
side = 10;
debug = 1;
toolDistance = 190;
trackWidth = 190;



clampPoints = clamp.pointCloud.Location;
xMiddle = (clamp.xmax +clamp.xmin)/2;
yMiddle = (clamp.ymax + clamp.ymin)/2;
middleTop = cubicDissect(clampPoints,xMiddle - side, xMiddle + side, yMiddle - side, yMiddle + side,clamp.zmin, clamp.zmax);
topCaptured = mean(middleTop);
smallerTop = cubicDissect(clampPoints,topCaptured(1)- twoSideDistance - side, topCaptured(1)- twoSideDistance + side, topCaptured(2) - side, topCaptured(2) + side,clamp.zmin, clamp.zmax);
smallerCaptured = mean(smallerTop);
biggerTop = cubicDissect(clampPoints,topCaptured(1) + twoSideDistance - side, topCaptured(1) + twoSideDistance + side, topCaptured(2) - side, topCaptured(2) + side,clamp.zmin, clamp.zmax);
biggerCaptured = mean(biggerTop);
%统一y坐标
smallerCaptured(2) = topCaptured(2);
biggerCaptured(2) = topCaptured(2);
%计算圆心和半径
[pc,r] = points2circle(topCaptured,smallerCaptured,biggerCaptured);
h = pc(3); % 高度
pos = [pc(1),pc(2)]; % 圆心位置
t=0:0.1:(2*pi);  % 圆滑性设置
t=[t,0];
matchingCircle = [(pos(1)+r*sin(t))', (pos(2)+ones(size(t)))',(h+r*cos(t))'];
[row,~] = find(matchingCircle(:,3) > clamp.panel);
topArc = matchingCircle(row,:);
topArc(:,2) = clamp.ymin;
topArc = sortrows(topArc,1,'ascend');
line1 = [clamp.xmin, clamp.ymin, topArc(1,3); topArc(1,:)];
line2 = [topArc(1,:); topArc(floor(size(topArc,1)/2),:); topArc(end,:)];
line3 = [topArc(end,:);clamp.xmax, clamp.ymin,topArc(end,3)];
downABC = [90,90,90];
topArcABC = [90,rad2deg(acos(-(line2(1,1)-pc(1))/r)),90; 90,rad2deg(acos(-(line2(2,1)-pc(1))/r)),90; 90,rad2deg(acos(-(line2(3,1)-pc(1))/r)),90;];
arcStartMark = [-80000, -80000, -80000, -80000, -80000, -80000];
arcEndMark = [-150000, -150000, -150000, -150000, -150000, -150000];


topModelTrack = [line1,[downABC;downABC];arcStartMark;line2,topArcABC;arcEndMark;line3,[downABC;downABC]];


totalNum = ceil((clamp.ymax - clamp.ymin)/trackWidth) + 1;

if mod((clamp.ymax - clamp.ymin), trackWidth)< trackWidth / 2
    totalNum = totalNum - 1;
end



firstNum = floor(totalNum/2);
secondNum = totalNum - firstNum;


topSprayTrack1 = zeros(size(topModelTrack,1)*firstNum,size(topModelTrack,2));
topSprayTrack2 = zeros(size(topModelTrack,1)*secondNum,size(topModelTrack,2));


yCurrent = topModelTrack(1,2);

for i=1:size(topModelTrack,1):firstNum*size(topModelTrack,1)
    if rem(i,2) ~= 0
    topSprayTrack1(i:i+8,:) = topModelTrack;
    else
    topSprayTrack1(i:i+8,:) = reverseTrack(topModelTrack);
    end
    topSprayTrack1(i:i+8,2) = yCurrent;
    yCurrent = yCurrent + trackWidth;
end


for i=1:size(topModelTrack,1):secondNum*size(topModelTrack,1)
    if rem(i,2) ~= 0
        topSprayTrack2(i:i+8,:) = topModelTrack;
    else
        topSprayTrack2(i:i+8,:) = reverseTrack(topModelTrack);
    end
    topSprayTrack2(i:i+8,2) = yCurrent;
    yCurrent = yCurrent + trackWidth;
end













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
close all;
pcshow(clamp.pointCloud);
% hold on
% pcshow(pointCloud(middleTop,"Color",'red'));
% hold on
% pcshow(pointCloud(topCaptured,"Color",'white'));
% 
% hold on
% pcshow(pointCloud(smallerTop,"Color",'green'));
% hold on
% pcshow(pointCloud(smallerCaptured,"Color",'white'));
% 
% hold on
% pcshow(pointCloud(biggerTop,"Color",'yellow'));
% hold on
% pcshow(pointCloud(biggerCaptured,"Color",'white'));
% 
% hold on
% pcshow(pointCloud(matchingCircle,"Color",'cyan'));
% hold on
% pcshow(pointCloud(topArc,"Color",'white'));





restoredTopSpraytrack1 = restoreTrack(topSprayTrack1);
restoredTopSpraytrack2 = restoreTrack(topSprayTrack2);
hold on 
plot3(restoredTopSpraytrack1(:,1), restoredTopSpraytrack1(:,2), restoredTopSpraytrack1(:,3),'.-blue')
hold on 
plot3(restoredTopSpraytrack2(:,1), restoredTopSpraytrack2(:,2), restoredTopSpraytrack2(:,3),'.-blue')



restoredHeadSprayTrack = restoreTrack(headSprayTrack);
restoredEndSprayTrack = restoreTrack(endSprayTrack);
hold on 
plot3(restoredHeadSprayTrack(:,1), restoredHeadSprayTrack(:,2), restoredHeadSprayTrack(:,3),'.-blue')
hold on 
plot3(restoredEndSprayTrack(:,1), restoredEndSprayTrack(:,2), restoredEndSprayTrack(:,3),'.-blue')





end















end