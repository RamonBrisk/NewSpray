function bottomMoreThan1200(clamp, radius, deltaZ)

% close all;
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


[wallSSSpraytrack, wallSLSpraytrack] = getWallTrack(clamp,matchingCircle,pc,r,-1.1,1.8,toolDistance,trackWidth);
[wallLSSpraytrack, wallLLSpraytrack] = getWallTrack(clamp,matchingCircle,pc,r,1.1,1.8,toolDistance,trackWidth);


%小侧面小端
xDirectionABC = [96 0 0];
model = [clamp.xmin-toolDistance, clamp.ymin, clamp.zmin; clamp.xmin - toolDistance, (clamp.ymax+clamp.ymin)/2,  clamp.zmin];
sideSSSprayTrack = getSideTrack(clamp, xDirectionABC, model, trackWidth);

%小侧面大端
model = [clamp.xmin-toolDistance, clamp.ymax, clamp.zmin; clamp.xmin - toolDistance, (clamp.ymax+clamp.ymin)/2,  clamp.zmin];
sideSLSprayTrack = getSideTrack(clamp, xDirectionABC, model, trackWidth);


%大侧面小端
xDirectionABC = [-84,0,0];
model = [clamp.xmax+toolDistance, clamp.ymin, clamp.zmin; clamp.xmax + toolDistance, (clamp.ymax+clamp.ymin)/2,  clamp.zmin];
sideLSSprayTrack = getSideTrack(clamp, xDirectionABC, model, trackWidth);

model = [clamp.xmax+toolDistance, clamp.ymax, clamp.zmin; clamp.xmax + toolDistance, (clamp.ymax+clamp.ymin)/2,  clamp.zmin];
sideLLSprayTrack = getSideTrack(clamp, xDirectionABC, model, trackWidth);






















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

    %     hold on
    %     pcshow(pointCloud(arcWall,"Color",'cyan'));


    % for i=1:size(allTracksDiscrete,3)
    % restoredAllTracksDiscrete = restoreTrack(allTracksDiscrete(:,:,i));
    % hold on
    % plot3(restoredAllTracksDiscrete(:,1), restoredAllTracksDiscrete(:,2), restoredAllTracksDiscrete(:,3),'.-blue')
    % end


    restoredWallSSSpraytrack = restoreTrack(wallSSSpraytrack);
    hold on
    plot3(restoredWallSSSpraytrack(:,1), restoredWallSSSpraytrack(:,2), restoredWallSSSpraytrack(:,3),'.-blue')

    restoredWallSLSpraytrack = restoreTrack(wallSLSpraytrack);
    hold on
    plot3(restoredWallSLSpraytrack(:,1), restoredWallSLSpraytrack(:,2), restoredWallSLSpraytrack(:,3),'.-blue')


    restoredWallLSSpraytrack = restoreTrack(wallLSSpraytrack);
    hold on
    plot3(restoredWallLSSpraytrack(:,1), restoredWallLSSpraytrack(:,2), restoredWallLSSpraytrack(:,3),'.-blue')

    restoredWallLLSpraytrack = restoreTrack(wallLLSpraytrack);
    hold on
    plot3(restoredWallLLSpraytrack(:,1), restoredWallLLSpraytrack(:,2), restoredWallLLSpraytrack(:,3),'.-blue')



    restoredsideSSSprayTrack = restoreTrack(sideSSSprayTrack);
    hold on
    plot3(restoredsideSSSprayTrack(:,1), restoredsideSSSprayTrack(:,2), restoredsideSSSprayTrack(:,3),'.-blue')

    restoredsideSLSprayTrack = restoreTrack(sideSLSprayTrack);
    hold on
    plot3(restoredsideSLSprayTrack(:,1), restoredsideSLSprayTrack(:,2), restoredsideSLSprayTrack(:,3),'.-blue')



    restoredsideLSSprayTrack = restoreTrack(sideLSSprayTrack);
    hold on
    plot3(restoredsideLSSprayTrack(:,1), restoredsideLSSprayTrack(:,2), restoredsideLSSprayTrack(:,3),'.-blue')

    restoredsideLLSprayTrack = restoreTrack(sideLLSprayTrack);
    hold on
    plot3(restoredsideLLSprayTrack(:,1), restoredsideLLSprayTrack(:,2), restoredsideLLSprayTrack(:,3),'.-blue')







end












end