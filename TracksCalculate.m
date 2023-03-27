
function TracksCalculate(PicQuantity,vehicleX,vehicleY,houseNum,imagePath,toolDistance,excess,fastBoot,xRotate,yRotate,zRotate)
tic
%需要合成的块数量
quantity = PicQuantity;
offset = 0;
src = zeros(1048576,3,quantity);RESULT = zeros(1048576*quantity,3);
casaNum = houseNum;
trackFolder = [int2str(casaNum),'\tracks\'];
status = mkdir(trackFolder);
delete("1.zip");
if fastBoot == 0
    for i=1:quantity
        path = [imagePath,'\',num2str(casaNum),' (',num2str(i+offset),').txt'];
        pcl = load(path);
        angleX = xRotate;
        rotateAngleX = [1,0,0;0, cos(deg2rad(angleX)), -sin(deg2rad(angleX));0,sin(deg2rad(angleX)),cos(deg2rad(angleX))];
        pcl = rotateAngleX * pcl';
        pcl = pcl';

        angleY = yRotate;
        rotateAngleY = [cos(deg2rad(angleY)),0,sin(deg2rad(angleY));0, 1, 0;-sin(deg2rad(angleY)), 0, cos(deg2rad(angleY))];
        pcl = rotateAngleY * pcl';
        pcl = pcl';

        
        % abc = rotateAngleX * rotateAngleY;


        angleZ = zRotate;
        rotateAngleZ = [cos(deg2rad(angleZ)),-sin(deg2rad(angleZ)),0;sin(deg2rad(angleZ)), cos(deg2rad(angleZ)), 0; 0, 0, 1];
        pcl = rotateAngleZ * pcl';
        pcl = pcl';


        % figure()
        % plot3(pcl(:,1),pcl(:,2),pcl(:,3),'.');
        picX=vehicleX;
        picY=vehicleY;
        dx = zeros(1,size(picX,2));
        dy = zeros(1,size(picY,2));
        for k=2:size(picX,2)
            dx(k)=picX(k)-picX(1);
            dy(k)=picY(k)-picY(1);
        end
        pcl(all(pcl==0,2),:) = [-50000];
        src(:,1,i)=-pcl(:,1)+dx(i);
        src(:,2,i)=-pcl(:,2)+dy(i);
        src(:,3,i)=-pcl(:,3);
        RESULT = [RESULT;src(:,:,i)];
    end
    RESULT(all(RESULT==0,2),:) = [];
    [row,~] = find(RESULT>20000);
    RESULT(row,:) = [];
    pcshow(RESULT);


    RESULT = cubicDissect(RESULT, -1380, 5600, -1850, 4760, -1780, -700);


    angleZ = 0;
    rotateAngleZ = [cos(deg2rad(angleZ)),-sin(deg2rad(angleZ)),0;sin(deg2rad(angleZ)), cos(deg2rad(angleZ)), 0; 0, 0, 1];
    RESULT = rotateAngleZ * RESULT';
    RESULT = RESULT';

    pcshow(RESULT);

%     RESULT(:,1) = RESULT(:,1)  + 175 + 600;
%     RESULT(:,2) = RESULT(:,2)  + 52 + 1560;
    ptCloud = pcdenoise(pointCloud(RESULT));
    % normals = pcnormals(ptCloud,100);
    pcwrite(ptCloud,'tempPC');
    figure()
    pcshow(ptCloud)
    disp("点云合并完成")
    toc
end

if fastBoot == 1
    tic
    ptCloud = pcread('tempPC.PLY');
    disp("读取合并后的点云成功");
    toc
end
figure()
pcshow(ptCloud);



tic;
interval = 190;
minDistance = 50;
minPoints = 9000;
[labels,numClusters] = pcsegdist(ptCloud,minDistance,'NumClusterPoints',minPoints);
lowerLock = 0;
for i=1:numClusters
    idxValidPoints = find(labels == i);
    %labelColorIndex = labels(idxValidPoints);
    segmentedPtCloud = select(ptCloud,idxValidPoints);
    zmin = min(segmentedPtCloud.Location(:,3));
    zmax = max(segmentedPtCloud.Location(:,3));
    hight = zmax - zmin;
    if(hight> 500)
        type = "下半件";
        lowerLock = lowerLock + 1;
    else
        type = "上半件";
    end
    Locks(i) = LockClamp(i,type,segmentedPtCloud);
end
disp("点云分类和切割完成")
toc;

fprintf("本批次工件共有%d个，其中有%d个下半件\n",numClusters,lowerLock);

tic;
yMinList = zeros(1,size(Locks,2),1);
sumMinimum = zeros(1,size(Locks,2),1);
sorted = [];
for id = 1:size(Locks,2)


    sumMinimum(id) = abs(Locks(id).xmin) + abs(Locks(id).ymin);
    yMinList(id) = Locks(id).ymin;
end
yMinListOri = yMinList;
sumMinimumOri = sumMinimum;
% plot(yMinList);
[~,index1st] = min(sumMinimum);
sorted = [sorted,index1st];
firstRowIndex = [];
for id = 1:size(Locks,2)
    if (id~=index1st)
        if (abs(Locks(index1st).ymin - Locks(id).ymin) < 500)
            firstRowIndex = [firstRowIndex,id];
        end
    end
end
firstRowxMinList = zeros(1,size(firstRowIndex,2),1);
for firstRowSortIndex = 1:size(firstRowIndex,2)
    firstRowxMinList(firstRowSortIndex) = Locks(firstRowIndex(firstRowSortIndex)).xmin;
end
for index1 = 1:size(firstRowIndex,2)
    [~,index] = min(firstRowxMinList);
    sorted = [sorted,firstRowIndex(index)];
    firstRowxMinList(index) = 10000000+index1;
end
%第一行排序完毕

if size(sorted,2) ~= size(Locks,2)
    sumMinimum(sorted) = [];
    yMinList(sorted) = [];
    [value,index2nd] = min(sumMinimum);
    Locks(index2nd).series = size(sorted,2) + 1;
    sorted = [sorted,index2nd];
    secondRowIndex = [];
    for id = 1:size(Locks,2)
        if (id~=index2nd)
            if (abs(Locks(index2nd).ymin - Locks(id).ymin) < 500)
                secondRowIndex = [secondRowIndex,id];
            end
        end
    end
    secondRowxMinList = zeros(1,size(secondRowIndex,2),1);
    for secondRowSortIndex = 1:size(secondRowIndex,2)
        secondRowxMinList(secondRowSortIndex) = Locks(secondRowIndex(secondRowSortIndex)).xmin;
    end
    for index1 = 1:size(secondRowIndex,2)
        [value,index] = min(secondRowxMinList);
        sorted = [sorted,secondRowIndex(index)];
        secondRowxMinList(index) = 10000000+index1;
    end
    %第二行排序完毕
    %第三行排序...
end


if size(sorted,2) ~= size(Locks,2)

    sumMinimum = sumMinimumOri;

    sumMinimum(sorted) = 10000000;

    [value,index3rd] = min(sumMinimum);
    Locks(index3rd).series = size(sorted,2) + 1;
    sorted = [sorted,index3rd];
    thirdRowIndex = [];
    for id = 1:size(Locks,2)
        if (id~=index3rd)
            if (abs(Locks(index3rd).ymin - Locks(id).ymin) < 500)
                thirdRowIndex = [thirdRowIndex,id];
            end
        end
    end
    thirdRowxMinList = zeros(1,size(thirdRowIndex,2),1);
    for thirdRowSortIndex = 1:size(thirdRowIndex,2)
        thirdRowxMinList(thirdRowSortIndex) = Locks(thirdRowIndex(thirdRowSortIndex)).xmin;
    end
    for index1 = 1:size(thirdRowIndex,2)
        [value,index] = min(thirdRowxMinList);
        sorted = [sorted,thirdRowIndex(index)];
        thirdRowxMinList(index) = 10000000+index1;
    end


end
%第三行排序完成

if size(sorted,2) ~= size(Locks,2)

    sumMinimum = sumMinimumOri;

    sumMinimum(sorted) = 10000000;

    [value,index4th] = min(sumMinimum);
    Locks(index4th).series = size(sorted,2) + 1;
    sorted = [sorted,index4th];
    fourthRowIndex = [];
    for id = 1:size(Locks,2)
        if (id~=index4th)
            if (abs(Locks(index4th).ymin - Locks(id).ymin) < 500)
                fourthRowIndex = [fourthRowIndex,id];
            end
        end
    end
    fourthRowxMinList = zeros(1,size(fourthRowIndex,2),1);
    for fourthRowSortIndex = 1:size(fourthRowIndex,2)
        fourthRowxMinList(fourthRowSortIndex) = Locks(fourthRowIndex(fourthRowSortIndex)).xmin;
    end
    for index1 = 1:size(fourthRowIndex,2)
        [value,index] = min(fourthRowxMinList);
        sorted = [sorted,fourthRowIndex(index)];
        fourthRowxMinList(index) = 10000000+index1;
    end


end

%第四行排序完成

%...
%...

toc;
disp("工件排序编号完成");
for i=1:size(Locks,2)
    Locks(sorted(i)).series = i;
end
sortedLocks = Locks;
for i = 1:size(Locks,2)
    sortedLocks(Locks(i).series) = Locks(i);
end
Locks = sortedLocks;
%figure()
locks4points = zeros(12,size(Locks,2));
modelInfo = zeros(1,size(Locks,2)+1);
modelInfo(1) = size(Locks,2);
modelInfo = zeros(2,size(Locks,2)+1);
modelInfo(1,1) = size(Locks,2);

for i = 1:size(Locks,2)
    %     colormap(hsv(i));
    %pcshow(Locks(i).pointCloud,"BackgroundColor",[0 0 0]);
    hold on;
    h = surf(Locks(i).surfaceX,Locks(i).surfaceY,Locks(i).surfaceZ);
    set(h,'FaceColor','b');
    hold on
    for paintingBaseNum = 1:size(Locks(i).paintingBase,1)
        h = Locks(i).panel; % 高度
        r = 150;  %半径
        pos = [Locks(i).paintingBase(paintingBaseNum,1),Locks(i).paintingBase(paintingBaseNum,2)]; % 圆心位置
        t=0:0.001:(2*pi);  % 圆滑性设置
        t=[t,0];
        hold on
        %         plot3(pos(1)+r*sin(t),pos(2)+r*cos(t), h*ones(size(t)))
    end
    % normals = pcnormals(Locks(i).pointCloud,300);
    % figure()
    % pcshow(Locks(i).pointCloud)
    % title('Estimated Normals of Point Cloud')
    % hold on
    % x = Locks(i).pointCloud.Location(1:200:end,1);
    % y = Locks(i).pointCloud.Location(1:200:end,2);
    % z = Locks(i).pointCloud.Location(1:200:end,3);
    % u = normals(1:200:end,1);
    % v = normals(1:200:end,2);
    % w = normals(1:200:end,3);
    % quiver3(x,y,z,u,v,w);
    % hold off
    %



    if Locks(i).type == "上半件"
        tic;

        if Locks(i).ymax - Locks(i).ymin <= 1200
            modelInfo(1,i+1) = 0;
        else
            modelInfo(1,i+1) = 1;
        end

        [row,col] = find(Locks(i).pointCloud.Location(:,3) > Locks(i).panel + 50);
        arc = Locks(i).pointCloud.Location(row,:);

        xmin = min(arc(:,1));
        xmax = max(arc(:,1));
        ymin = min(arc(:,2));
        ymax = max(arc(:,2));
        zmin = min(arc(:,3));
        zmax = max(arc(:,3));
        p1 = [xmin,ymin,zmin];
        p2 = [(xmin+xmax)/2,ymin,zmax];
        p3 = [xmax,ymin,zmin];
        [pc,r] = points2circle(p1,p2,p3);
        delta = interval;
        %pcshow(arc);
        %hold on
        %pcshow(pc,'*');
        %hold on
        h = pc(3); % 高度
        % r = r;  %半径
        pos = [pc(1),pc(2)]; % 圆心位置
        t=0:0.1:(2*pi);  % 圆滑性设置
        t=[t,0];
        matchingCircle = [(pos(1)+r*sin(t))', (pos(2)+ones(size(t)))',(h+r*cos(t))'];
        [row,~] = find(matchingCircle(:,3) > Locks(i).panel);
        [headFaceArcRow, ~]= find(matchingCircle(:,3) > Locks(i).zmin -30);

        headFaceArc = matchingCircle(headFaceArcRow,:);
        headFaceArc(:,2) = headFaceArc(:,2) - toolDistance;
        %         plot3(headFaceArc(:,1),headFaceArc(:,2),headFaceArc(:,3),'.');
        allTimes = ceil((ymax - ymin) / interval) + 1;
        if mod((Locks(i).ymax - Locks(i).ymin), interval)< interval / 2

            allTimes = allTimes - 1;

        end



        for time=1:allTimes
            matchingArc(:,:,time) = matchingCircle(row,:);
            matchingArc(:,2,time) = matchingArc(:,2,time) + delta - interval;
            xminArc = min(matchingArc(:,1,time));
            xmaxArc = max(matchingArc(:,1,time));
            arcLine(:,1:3,time) = [Locks(i).xmin - excess, ymin + delta - interval,Locks(i).panel; xminArc, ymin + delta - interval,Locks(i).panel; xmaxArc, ymin + delta - interval,Locks(i).panel; Locks(i).xmax + excess, ymin + delta - interval,Locks(i).panel];
            for t = 1:size(arcLine(:,:,time),1)
                arcLine(t,4:6,time) = [90,90,90];
            end

            delta = delta + interval;

            % for time = 1:5
            % plot3(arcLine(:,1,time),arcLine(:,2,time),arcLine(:,3,time),'-*');
            %                    hold on
            % end

            %             plot3(arcLine(:,1,time),arcLine(:,2,time),arcLine(:,3,time),'-*');
            %             hold on
            %             plot3(matchingArc(:,1,time),matchingArc(:,2,time),matchingArc(:,3,time),'.');
            %             hold on
        end
        endFaceArc = matchingCircle(headFaceArcRow,:);
        endFaceArc(:,2) = Locks(i).ymax + toolDistance;
        %         plot3(endFaceArc(:,1),endFaceArc(:,2),endFaceArc(:,3),'.');
        delta = interval;

        %面积计算
        forehead = (xminArc - Locks(i).xmin) * (Locks(i).panel - Locks(i).zmin);
        headFaceArea = forehead * 2;
        oneSide = (Locks(i).ymax - Locks(i).ymin) * (Locks(i).panel - Locks(i).zmin);
        sideArea = oneSide * 2;
        curve = 2 * r * asin((xmaxArc - xminArc) / (2 * r));
        arcArea = curve * (Locks(i).ymax - Locks(i).ymin);
        oneFlat = (xminArc - Locks(i).xmin) * (Locks(i).ymax - Locks(i).ymin);
        Area = headFaceArea + sideArea + arcArea + 2 * oneFlat;
        Locks(i).area = Area;
        fprintf("编号为%d的工件喷涂面积为%d \n",i,Locks(i).area);
        modelInfo(2,i+1) = Locks(i).area;
        %实际路线规划点位1 左侧立面
        delta = interval;
        safetySpot = [];
        times = ceil((Locks(i).panel - Locks(i).zmin) / delta);


        if Locks(i).ymax - Locks(i).ymin <= 1200


            track1 = safetySpot;
            for time = 1:times
                spot1 = [Locks(i).xmin-toolDistance, Locks(i).ymax + excess,Locks(i).zmin + 110 + delta-interval,96,0,0];
                spot2 = [ Locks(i).xmin-toolDistance, Locks(i).ymin - excess,Locks(i).zmin + 110 + delta-interval,96,0,0];
                delta = delta+interval;
                if mod(time,2)
                    track1 = [track1;spot2;spot1];
                else
                    track1 = [track1;spot1;spot2];
                end
            end
            track1 = [track1;safetySpot];


        else


            track1 = safetySpot;
            for time = 1:times
                spot1 = [Locks(i).xmin-toolDistance, Locks(i).ymax + excess,Locks(i).zmin + 110 + delta-interval,96,0,0];
                spot2 = [ Locks(i).xmin-toolDistance, (Locks(i).ymax + Locks(i).ymin)/2,Locks(i).zmin + 110 + delta-interval,96,0,0];
                delta = delta+interval;
                if mod(time,2)
                    track1 = [track1;spot2;spot1];
                else
                    track1 = [track1;spot1;spot2];
                end
            end
            track1 = [track1;safetySpot];



            track7 = safetySpot;
            for time = 1:times
                spot1 = [Locks(i).xmax+toolDistance, Locks(i).ymin - excess,Locks(i).zmin + 110 + delta-interval,96,0,0];
                spot2 = [Locks(i).xmax+toolDistance, (Locks(i).ymax + Locks(i).ymin)/2,Locks(i).zmin + 110 + delta-interval,96,0,0];
                delta = delta+interval;
                if mod(time,2)
                    track7 = [track7;spot2;spot1];
                else
                    track7 = [track7;spot1;spot2];
                end
            end
            track7 = [track7;safetySpot];
        end





        %hold on
        %plot3(track1(:,1),track1(:,2),track1(:,3),'*-');
        %实际路线规划点位1 前端面
        track2 = safetySpot;

        row = size(headFaceArc,1);
        headFaceArcVector = zeros(row,6);
        for t=1:row
            headFaceArcVector(t,4:6) = [176,0,40];
        end
        headFaceArcVector(:,1:3) = sortrows(headFaceArc,1,'ascend');


        track2 = [track2;headFaceArcVector];
        %hold on
        track2 = [track2;safetySpot];

        [row,~] = find(track2(:,3)<track2(1,3)+40);
        track2(row,:) = [];





        track2 = line2curve(track2);








        %         [m,n] = size(track2);
        %         gesture = zeros(m,n);
        %         for t=1:m
        %             gesture(t,:) = [-180,90,0];
        %         end
        %         track2 = [track2 gesture];
        %
        %

        %plot3(track2(:,1),track2(:,2),track2(:,3),'*-');
        %实际路线规划点位2 上表面1
        track4 = safetySpot;
        firstTimes = ceil((ymax - ymin) / 2 / interval);

        if Locks(i).ymax - Locks(i).ymin <= 1200
            firstTimes = ceil((ymax - ymin) / interval);
        end









        %         确定一条顶面轨迹的角度矩阵

        % plot3(sortedHeadFaceArc(:,1),sortedHeadFaceArc(:,2),sortedHeadFaceArc(:,3));
        % hold on
        % plot3(pc(1),pc(2),pc(3),'r*');


        sortedHeadFaceArc = sortrows(matchingArc(:,:,1),1,'ascend');
        row = size(sortedHeadFaceArc,1);
        matchingArcVector = zeros(row,3);
        for t=1:row
            %matchingArcVector(t,:) = [0,rad2deg(real(acos((sortedHeadFaceArc(t,3,1)-h)/r))),90];
            matchingArcVector(t,:) = [90,rad2deg(acos(-(sortedHeadFaceArc(t,1,1)-pc(1))/r)),90];
        end


        for time = 1:firstTimes
            if mod(time,2)
                track4 = [track4;arcLine(4,:,time);arcLine(3,:,time)];
                reverseVector = sortrows(matchingArcVector,2,'descend');
                track4 = [track4;line2curve([sortrows(matchingArc(:,:,time),1,'descend'),reverseVector])];
                 track4 = [track4;arcLine(2,:,time);arcLine(1,:,time)];
            else
                track4 = [track4;arcLine(1,:,time);arcLine(2,:,time)];
                track4 = [track4;line2curve([sortrows(matchingArc(:,:,time),1,'ascend'),matchingArcVector])];
                track4 = [track4;arcLine(3,:,time);arcLine(4,:,time)];
            end


        end
        track4 = [track4;safetySpot];
        %hold on



        %plot3(track3(:,1),track3(:,2),track3(:,3),'*-');
        %实际路线规划点位2 上表面2
        track6 = safetySpot;

        if Locks(i).ymax - Locks(i).ymin > 1200
            secondTimes = allTimes - firstTimes + 1;
            for time = allTimes:-1:secondTimes
                if mod(time,2)
                    track6 = [track6;arcLine(1,:,time);arcLine(2,:,time)];
                    track6 = [track6;line2curve([sortrows(matchingArc(:,:,time),1,'ascend'),matchingArcVector])];
                    track6 = [track6;arcLine(3,:,time);arcLine(4,:,time)];
                else
                    track6 = [track6;arcLine(4,:,time);arcLine(3,:,time)];
                    reverseVector = sortrows(matchingArcVector,2,'descend');
                    reverseVector(:,2) = reverseVector(:,2);
                    track6 = [track6;line2curve([sortrows(matchingArc(:,:,time),1,'descend'),reverseVector])];
                    track6 = [track6;arcLine(2,:,time);arcLine(1,:,time)];
                end
            end
            %track6 = [track6;track3(size(track3,1)-1,:)];
            track6 = [track6;safetySpot];
            %         hold on
            %         plot3(track4(:,1),track4(:,2),track4(:,3),'*-');


            track6(:,3) = track6(:,3) + toolDistance;
            track4(:,3) = track4(:,3) + toolDistance;

        end





        %实际路线规划点位3 右边侧立面
        delta = interval;
        times = ceil((Locks(i).panel - Locks(i).zmin) / delta);


        if Locks(i).ymax - Locks(i).ymin <= 1200
            track5 = safetySpot;
            for time = 1:times
                spot1 = [Locks(i).xmax+toolDistance, Locks(i).ymax + excess,Locks(i).zmin + 180 + delta-interval,-84,0,0];
                spot2 = [ Locks(i).xmax+toolDistance, Locks(i).ymin - excess,Locks(i).zmin + 180 + delta-interval,-84,0,0];
                delta = delta+interval;
                if mod(time,2)
                    track5 = [track5;spot2;spot1];
                else
                    track5 = [track5;spot1;spot2];
                end
            end
            track5 = [track5;safetySpot];

        else
            track5 = safetySpot;
            for time = 1:times
                spot1 = [Locks(i).xmax+toolDistance,  Locks(i).ymin-excess,Locks(i).zmin + 180 + delta-interval,-84,0,0];
                spot2 = [ Locks(i).xmax+toolDistance, (Locks(i).ymax+Locks(i).ymin)/2,Locks(i).zmin + 180 + delta-interval,-84,0,0];
                delta = delta+interval;
                if mod(time,2)
                    track5 = [track5;spot2;spot1];
                else
                    track5 = [track5;spot1;spot2];
                end
            end
            track5 = [track5;safetySpot];


            track8 = safetySpot;
            for time = 1:times
                spot1 = [Locks(i).xmin-toolDistance, Locks(i).ymax+excess,Locks(i).zmin + 180 + delta-interval,-84,0,0];
                spot2 = [ Locks(i).xmin-toolDistance, (Locks(i).ymax+Locks(i).ymin)/2,Locks(i).zmin + 180 + delta-interval,-84,0,0];
                delta = delta+interval;
                if mod(time,2)
                    track8 = [track8;spot2;spot1];
                else
                    track8 = [track8;spot1;spot2];
                end
            end
            track8 = [track8;safetySpot];


        end





        %         hold on
        %         %plot3(track5(:,1),track5(:,2),track5(:,3),'*-');

        %
        %         %实际路线规划点位3 后端面
        %         delta = interval;
        %         safetySpot = [Locks(i).xmin,Locks (i).ymin,-1000];
        %         times = ceil((Locks(i).panel - Locks(i).zmin) / delta) + 1;
        %         track6 = safetySpot;
        %         for time = 1:times
        %             spot1 = [Locks(i).xmax+200, Locks(i).ymax + 190,Locks(i).zmin + delta-interval];
        %             spot2 = [ Locks(i).xmax+200, Locks(i).ymin - 190,Locks(i).zmin + delta-interval];
        %             delta = delta+interval;
        %             if mod(time,2)
        %                 track6 = [track6;spot2;spot1];
        %             else
        %                 track6 = [track6;spot1;spot2];
        %             end
        %         end
        %         track6 = [track6;safetySpot];
        %         hold on
        %         %plot3(track6(:,1),track6(:,2),track6(:,3),'*-');



        track3 = safetySpot;
        row = size(endFaceArc,1);
        endFaceArcVector = zeros(row,6);
        for t=1:row
            endFaceArcVector(t,4:6) = [5,4,-40];
        end
        endFaceArcVector(:,1:3) = sortrows(endFaceArc,1,'ascend');
        track3 = [track3;endFaceArcVector];
        %hold on
        track3 = [track3;safetySpot];
        [row,~] = find(track3(:,3)<track3(1,3)+40);
        track3(row,:) = [];
        track3 = line2curve(track3);

        %plot3(track3(:,1),track6(:,2),track6(:,3),'*-');

        %
        %     figure
        %     x = track2(1:100:end,1);
        %     y = track2(1:100:end,2);
        %     z = track2(1:100:end,3);
        %     u = track2(1:100:end,4);
        %     v = track2(1:100:end,5);
        %     w = track2(1:100:end,6);
        %     quiver3(x,y,z,u,v,w);
        %
        %
        % track1(:,1:3) = -track1(:,1:3);
        % track2(:,1:3) = -track2(:,1:3);
        % track3(:,1:3) = -track3(:,1:3);
        % track4(:,1:3) = -track4(:,1:3);
        % track5(:,1:3) = -track5(:,1:3);
        % track6(:,1:3) = -track6(:,1:3);

        %
        %         track2MinZ = min(track2(:,3));
        %         [row,~] = find(track2(:,3) < track2MinZ + 220);
        %         track2(row,:) = [];
        %
        %         track3MinZ = min(track3(:,3));
        %         [row,~] = find(track3(:,3) < track3MinZ + 220);
        %         track3(row,:) = [];

        track6(:,6) = 90;


        %         figure
        %         hold on;
        %         plot3(track1(:,1),track1(:,2),track1(:,3),'*-');hold on
        %         plot3(track2(:,1),track2(:,2),track2(:,3),'*-');hold on
        %         plot3(track3(:,1),track3(:,2),track3(:,3),'*-');hold on
        %         plot3(track4(:,1),track4(:,2),track4(:,3),'*-');hold on
        %         plot3(track5(:,1),track5(:,2),track5(:,3),'*-');hold on
        %         plot3(track6(:,1),track6(:,2),track6(:,3),'*-');hold on
        %
        %
        %     hold on
        %     x = track6(1:3:end,1);
        %     y = track6(1:3:end,2);
        %     z = track6(1:3:end,3);
        %     u = track6(1:3:end,5);
        %     u = u - 45;
        %     v = zeros(size(u,1),1);
        %     v = v + 0;
        %     w = zeros(size(u,1),1);
        %     w = w + 0;
        %     %     v = track6(1:3:end,5);
        %     %     w = track6(1:3:end,6);
        %     quiver3(x,y,z,u,v,w);
        %
        %     	track1(:,1) = Locks(i).paintingBase(1,1) - track1(:,1);
        %         track1(:,2) = Locks(i).paintingBase(1,2) - track1(:,2);
        %         track2(:,1) = Locks(i).paintingBase(2,1) - track2(:,1);
        %         track2(:,2) = Locks(i).paintingBase(2,2) - track2(:,2);
        %         track3(:,1) = Locks(i).paintingBase(4,1) - track3(:,1);
        %         track3(:,2) = Locks(i).paintingBase(4,2) - track3(:,2);
        %         track4(:,1) = Locks(i).paintingBase(2,1) - track4(:,1);
        %         track4(:,2) = Locks(i).paintingBase(2,2) - track4(:,2);
        %         track5(:,1) = Locks(i).paintingBase(3,1) - track5(:,1);
        %         track5(:,2) = Locks(i).paintingBase(3,2) - track5(:,2);
        %         track6(:,1) = Locks(i).paintingBase(4,1) - track6(:,1);
        %         track6(:,2) = Locks(i).paintingBase(4,2) - track6(:,2);




        if Locks(i).ymax - Locks(i).ymin <= 1200
            track1(:,1) = Locks(i).paintingBase(1,1) - track1(:,1);
            track1(:,2) = -Locks(i).paintingBase(1,2) + track1(:,2);
            track2(:,1) = Locks(i).paintingBase(2,1) - track2(:,1);
            track2(:,2) = -Locks(i).paintingBase(2,2) + track2(:,2);
            track3(:,1) = Locks(i).paintingBase(5,1) - track3(:,1);
            track3(:,2) = -Locks(i).paintingBase(5,2) + track3(:,2);
            track4(:,1) = Locks(i).paintingBase(4,1) - track4(:,1);
            track4(:,2) = -Locks(i).paintingBase(4,2) + track4(:,2);
            track5(:,1) = Locks(i).paintingBase(3,1) - track5(:,1);
            track5(:,2) = -Locks(i).paintingBase(3,2) + track5(:,2);




            track1(:,[1;2]) = track1(:,[2;1]);
            track2(:,[1;2]) = track2(:,[2;1]);
            track3(:,[1;2]) = track3(:,[2;1]);
            track4(:,[1;2]) = track4(:,[2;1]);
            track5(:,[1;2]) = track5(:,[2;1]);



            track1(:,3) =  620 - track1(:,3);
            track2(:,3) =  620 - track2(:,3);
            track3(:,3) =  620 - track3(:,3);
            track4(:,3) =  620 - track4(:,3);
            track5(:,3) =  620 - track5(:,3);


            track2 = curveRemark(track2,1,0);
            track3 = curveRemark(track3,1,0);
            track4 = curveRemark(track4,1,0);


            track2 = [track2(2,:);track2];
            track3 = [track3(2,:);track3];



            fileName =  [trackFolder,'Foundation-',int2str(i),'-','.txt'];
            paintingBase = Locks(i).paintingBase;
            save(fileName,'paintingBase','-ascii');
            %此处输出机器人指令
            %工件序号-轨迹序号
            for spotNum = 1:5
                switch spotNum
                    case 1
                        fileName =  [trackFolder,int2str(i),'-','1.txt'];
                        save(fileName,'track1','-ascii');
                    case 2
                        fileName =  [trackFolder,int2str(i),'-','2.txt'];
                        save(fileName,'track2','-ascii');
                    case 3
                        fileName =  [trackFolder,int2str(i),'-','5.txt'];
                        save(fileName,'track5','-ascii');
                    case 4
                        fileName =  [trackFolder,int2str(i),'-','4.txt'];
                        save(fileName,'track4','-ascii');
                    case 5
                        fileName =  [trackFolder,int2str(i),'-','3.txt'];
                        save(fileName,'track3','-ascii');

                end
            end










        else

            track1(:,1) = Locks(i).paintingBase(1,1) - track1(:,1);
            track1(:,2) = -Locks(i).paintingBase(1,2) + track1(:,2);
            track2(:,1) = Locks(i).paintingBase(2,1) - track2(:,1);
            track2(:,2) = -Locks(i).paintingBase(2,2) + track2(:,2);
            track3(:,1) = Locks(i).paintingBase(6,1) - track3(:,1);
            track3(:,2) = -Locks(i).paintingBase(6,2) + track3(:,2);
            track4(:,1) = Locks(i).paintingBase(4,1) - track4(:,1);
            track4(:,2) = -Locks(i).paintingBase(4,2) + track4(:,2);
            track5(:,1) = Locks(i).paintingBase(3,1) - track5(:,1);
            track5(:,2) = -Locks(i).paintingBase(3,2) + track5(:,2);
            track6(:,1) = Locks(i).paintingBase(5,1) - track6(:,1);
            track6(:,2) = -Locks(i).paintingBase(5,2) + track6(:,2);
            track7(:,1) = Locks(i).paintingBase(7,1) - track7(:,1);
            track7(:,2) = -Locks(i).paintingBase(7,2) + track7(:,2);
            track8(:,1) = Locks(i).paintingBase(8,1) - track8(:,1);
            track8(:,2) = -Locks(i).paintingBase(8,2) + track8(:,2);


            track6(:,[1;2]) = track6(:,[2;1]);
            track6 = curveRemark(track6,1,0);
            track7(:,[1;2]) = track7(:,[2;1]);
            track7 = curveRemark(track7,1,0);
            track8(:,[1;2]) = track8(:,[2;1]);
            track8 = curveRemark(track8,1,0);


            track1(:,[1;2]) = track1(:,[2;1]);
            track2(:,[1;2]) = track2(:,[2;1]);
            track3(:,[1;2]) = track3(:,[2;1]);
            track4(:,[1;2]) = track4(:,[2;1]);
            track5(:,[1;2]) = track5(:,[2;1]);
            track6(:,[1;2]) = track6(:,[2;1]);
            track7(:,[1;2]) = track7(:,[2;1]);
            track8(:,[1;2]) = track8(:,[2;1]);


            track1(:,3) =  620 - track1(:,3);
            track2(:,3) =  620 - track2(:,3);
            track3(:,3) =  620 - track3(:,3);
            track4(:,3) =  620 - track4(:,3);
            track5(:,3) =  620 - track5(:,3);
            track6(:,3) =  620 - track6(:,3);
            track7(:,3) =  620 - track7(:,3);
            track8(:,3) =  620 - track8(:,3);




            track2 = curveRemark(track2,1,0);
            track3 = curveRemark(track3,1,0);
            track4 = curveRemark(track4,1,0);
            track6 = curveRemark(track6,1,0);


            track2 = [track2(2,:);track2];
            track4 = [track4(2,:);track4];



            fileName =  [trackFolder,'Foundation-',int2str(i),'-','.txt'];
            paintingBase = Locks(i).paintingBase;
            save(fileName,'paintingBase','-ascii');
            %此处输出机器人指令
            %工件序号-轨迹序号
            for spotNum = 1:8
                switch spotNum
                    case 1
                        fileName =  [trackFolder,int2str(i),'-','1.txt'];
                        save(fileName,'track1','-ascii');
                    case 2
                        fileName =  [trackFolder,int2str(i),'-','2.txt'];
                        save(fileName,'track2','-ascii');
                    case 3
                        fileName =  [trackFolder,int2str(i),'-','5.txt'];
                        save(fileName,'track5','-ascii');
                    case 4
                        fileName =  [trackFolder,int2str(i),'-','4.txt'];
                        save(fileName,'track6','-ascii');
                    case 5
                        fileName =  [trackFolder,int2str(i),'-','6.txt'];
                        save(fileName,'track3','-ascii');
                    case 6
                        fileName =  [trackFolder,int2str(i),'-','3.txt'];
                        save(fileName,'track4','-ascii');

                    case 7
                        fileName =  [trackFolder,int2str(i),'-','7.txt'];
                        save(fileName,'track7','-ascii');
                    case 8
                        fileName =  [trackFolder,int2str(i),'-','8.txt'];
                        save(fileName,'track8','-ascii');


                end
            end






            %         hold on;
            %         plot3(track1(:,1),track1(:,2),track1(:,3),'*-');hold on
            %         plot3(track2(:,1),track2(:,2),track2(:,3),'*-');hold on
            %         plot3(track3(:,1),track3(:,2),track3(:,3),'*-');hold on
            %         plot3(track4(:,1),track4(:,2),track4(:,3),'*-');hold on
            %         plot3(track5(:,1),track5(:,2),track5(:,3),'*-');hold on
            %         plot3(track6(:,1),track6(:,2),track6(:,3),'*-');hold on



















        end


        %
        % trace1 = sortrows(headFaceArc,1,'ascend');
        % spot1 = [Locks(i).xmax + 200,Locks(i).ymin - 190,Locks(i).zmin ];
        % spot2 = [Locks(i).xmax + 200,Locks(i).ymax + 190,Locks(i).zmin ];
        % paintingTrack = [trace1;spot1;spot2];
        % hold on
        % plot3(paintingTrack(:,1),paintingTrack(:,2),paintingTrack(:,3));
        %
        %
        clear matchingArc;
        massage = [int2str(i),"工件(上半件轨迹计算完成)"];
        disp(massage)
        toc;

    end



    if Locks(i).type == "下半件"
        tic;
        if Locks(i).ymax - Locks(i).ymin <= 1200
            modelInfo(1,i+1) = 2;
        else
            modelInfo(1,i+1) = 3;
        end

        [row,col] = find(Locks(i).pointCloud.Location(:,3) > Locks(i).panel + 10);
        arc = Locks(i).pointCloud.Location(row,:);


        %         arc = pcde0noise(pointCloud(arc));
        %         arc = arc.Location;
        %         arc = sortrows(c,1,'descend');
        %         ymin = min(arc(:,2));
        %         ymax = max(arc(:,2));
        %         zmin = min(arc(:,3));
        %         zmax = max(arc(:,3));0
        %
        %         figure()
        %         pcshow(arc,'*');










        xmedia = (Locks(i).xmin + Locks(i).xmax) / 2;
        range = 20;
        [row,~] = find(Locks(i).pointCloud.Location(:,1)>xmedia - range & Locks(i).pointCloud.Location(:,1)<xmedia + range);
        xSelectedPoints = Locks(i).pointCloud.Location(row,:);
        [row,~] = find(Locks(i).pointCloud.Location(:,2)>Locks(i).ymax - range*2 & Locks(i).pointCloud.Location(:,2)<Locks(i).ymax);
        ySelectedPoints = Locks(i).pointCloud.Location(row,:);
        %         hold on
        %         plot3(xSelectedPoints(:,1),xSelectedPoints(:,2),xSelectedPoints(:,3),'*');
        %         hold on
        %         plot3(ySelectedPoints(:,1),ySelectedPoints(:,2),ySelectedPoints(:,3),'*');


        [c, ~, ~] = intersect(xSelectedPoints,ySelectedPoints,'rows');

        if isempty(c)
            disp('警告，识别错误');
            continue;
        end


        zminList = sortrows(c,3,'ascend');
        p1 = zminList(ceil(size(zminList,1) / 100),:);
        hold on
        plot3(p1(1),p1(2),p1(3),'w*');


        xleft = xmedia - 280;
        [row,~] = find(Locks(i).pointCloud.Location(:,1)>xleft - range & Locks(i).pointCloud.Location(:,1)<xleft + range);
        xSelectedPoints = Locks(i).pointCloud.Location(row,:);

        [row,~] = find(abs(Locks(i).pointCloud.Location(:,2) - p1(2)) < 10);
        ySelectedPoints = Locks(i).pointCloud.Location(row,:);

        %         hold on
        %         plot3(xSelectedPoints(:,1),xSelectedPoints(:,2),xSelectedPoints(:,3),'.');
        %         hold on
        %         plot3(ySelectedPoints(:,1),ySelectedPoints(:,2),ySelectedPoints(:,3),'.');


        [c, ~, ~] = intersect(xSelectedPoints,ySelectedPoints,'rows');
        zminList = sortrows(c,3,'ascend');
        p2 = zminList(ceil(size(zminList,1) / 100),:);

        p2(2) = p1(2);

        hold on
        plot3(p2(1),p2(2),p2(3),'w*');

        xright = xmedia + 280;
        [row,~] = find(Locks(i).pointCloud.Location(:,1)>xright - range & Locks(i).pointCloud.Location(:,1)<xright + range);
        xSelectedPoints = Locks(i).pointCloud.Location(row,:);
        [row,~] = find(abs(Locks(i).pointCloud.Location(:,2) - p1(2)) < 1);
        ySelectedPoints = Locks(i).pointCloud.Location(row,:);
        [c, ~, ~] = intersect(xSelectedPoints,ySelectedPoints,'rows');
        zminList = sortrows(c,3,'ascend');
        p3 = zminList(ceil(size(zminList,1) / 100),:);
        p3(2) = p1(2);
        hold on
        plot3(p3(1),p3(2),p3(3),'w*');


        % 第一次圆匹配
        [pc,r] = points2circle(p1,p2,p3);
        r1 = r;
        delta = interval;
        %hold on
        %pcshow(pc,'*');
        %hold on
        h = pc(3); % 高度
        % r = r;  %半径
        pos = [pc(1),pc(2)]; % 圆心位置
        t=0:0.001:(2*pi);  % 圆滑性设置
        t=[t,0];
        matchingCircle = [(pos(1)+r*sin(t))', (pos(2)+ones(size(t)))',(h+r*cos(t))'];
        firstMatchingCircle = matchingCircle;
        testCircle = matchingCircle;
        testCircle(:,2) = testCircle(:,2) - 300;

        plot3(matchingCircle(:,1),matchingCircle(:,2),matchingCircle(:,3),'.');
        plot3(testCircle(:,1),testCircle(:,2),testCircle(:,3),'.');
        %         hold on
        [row,~] = find(matchingCircle(:,3) > Locks(i).panel - 10);
        [headFaceArcRow, ~]= find(matchingCircle(:,3) > Locks(i).zmin -30);


        headFaceArc = matchingCircle(headFaceArcRow,:);
        headFaceArc(:,2) = Locks(i).ymin;
        headFaceArc(:,2) = headFaceArc(:,2) - toolDistance;

        %         plot3(headFaceArc(:,1),headFaceArc(:,2),headFaceArc(:,3),'.');



        [row,~] = find(testCircle(:,3) > Locks(i).panel);
        testCurve = testCircle(row,:);
        testCurve = sortrows(testCurve,1,'ascend');
        hold on
        plot3(testCurve(:,1),testCurve(:,2),testCurve(:,3),'.');

        near = [];
        for xpos = 200:fix(size(testCurve,1)/2)

            range = 10;
            [row,~] = find(Locks(i).pointCloud.Location(:,1)>testCurve(xpos,1) - range & Locks(i).pointCloud.Location(:,1)<testCurve(xpos,1) + range);
            xSelectedPoints = Locks(i).pointCloud.Location(row,:);
            hold on
            %             plot3(xSelectedPoints(:,1),xSelectedPoints(:,2),xSelectedPoints(:,3),'.');
            [row,~] = find(Locks(i).pointCloud.Location(:,2) > (testCurve(xpos,2) - range) & Locks(i).pointCloud.Location(:,2) < (testCurve(xpos,2) + range));
            ySelectedPoints = Locks(i).pointCloud.Location(row,:);
            hold on
            %             plot3(ySelectedPoints(:,1),ySelectedPoints(:,2),ySelectedPoints(:,3),'.');
            [row,~] = find(Locks(i).pointCloud.Location(:,3) > (testCurve(xpos,3) - range) & Locks(i).pointCloud.Location(:,3) < (testCurve(xpos,3) + range));
            zSelectedPoints = Locks(i).pointCloud.Location(row,:);
            %             hold on
            %             plot3(zSelectedPoints(:,1),zSelectedPoints(:,2),zSelectedPoints(:,3),'.');
            [c, ~, ~] = intersect(xSelectedPoints,ySelectedPoints,'rows');
            [c, ~, ~] = intersect(c,zSelectedPoints,'rows');
            if isempty(c)
            else
                near = [near;c];
            end
        end


        if isempty(near)

            testCurve = sortrows(testCurve,3,"descend");
            [row,~] = find(Locks(i).pointCloud.Location(:,1)<testCurve(1,1));
            xSelectedPoints = Locks(i).pointCloud.Location(row,:);
            hold on
            %             plot3(xSelectedPoints(:,1),xSelectedPoints(:,2),xSelectedPoints(:,3),'r.');
            [row,~] = find(Locks(i).pointCloud.Location(:,2) > (testCurve(1,2) - 5) & Locks(i).pointCloud.Location(:,2) < (testCurve(1,2) + 5));
            ySelectedPoints = Locks(i).pointCloud.Location(row,:);
            hold on
            %             plot3(ySelectedPoints(:,1),ySelectedPoints(:,2),ySelectedPoints(:,3),'g.');
            [row,~] = find(Locks(i).pointCloud.Location(:,3) > (testCurve(1,3) - 5) & Locks(i).pointCloud.Location(:,3) < (testCurve(1,3) + 5));
            zSelectedPoints = Locks(i).pointCloud.Location(row,:);
            hold on
            %             plot3(zSelectedPoints(:,1),zSelectedPoints(:,2),zSelectedPoints(:,3),'w.');
            [c, ~, ~] = intersect(xSelectedPoints,ySelectedPoints,'rows');
            [c, ~, ~] = intersect(c,zSelectedPoints,'rows');

            c = sortrows(c,1,"ascend");
            near = c(1,:);



        end

        hold on
        plot3(near(:,1),near(:,2),near(:,3),'ow');
        xSaltatory = sortrows(near,3,'ascend');
        xSaltatory = xSaltatory(1,:);
        hold on
        plot3(xSaltatory(1),xSaltatory(2),xSaltatory(3),'*g');

        zmedia = xSaltatory(1,3) + 100;
        [row,~] = find(abs(Locks(i).pointCloud.Location(:,3) - zmedia) < 2);
        zSelectedPoints = Locks(i).pointCloud.Location(row,:);
        %hold on
        %         plot3(zSelectedPoints(:,1),zSelectedPoints(:,2),zSelectedPoints(:,3),'.');
        [row,~] = find(abs(Locks(i).pointCloud.Location(:,2) - xSaltatory(1,2)) < 2);
        ySelectedPoints = Locks(i).pointCloud.Location(row,:);
        %hold on
        %         plot3(ySelectedPoints(:,1),ySelectedPoints(:,2),ySelectedPoints(:,3),'.');
        [c, ~, ~] = intersect(zSelectedPoints,ySelectedPoints,'rows');
        xminList = sortrows(c,1,'ascend');
        p2 = mean(xminList);
        [rightSideIndex,~] = find(xminList(:,1)<p2(1));
        xminList = xminList(rightSideIndex,:);
        p2 = mean(xminList);
        p2(2) = xSaltatory(2);
        hold on
        plot3(p2(1),p2(2),p2(3),'*g');

        ztop = xSaltatory(1,3) + 200;
        [row,~] = find(abs(Locks(i).pointCloud.Location(:,3) - ztop) < 8);
        zSelectedPoints = Locks(i).pointCloud.Location(row,:);
        hold on
        plot3(zSelectedPoints(:,1),zSelectedPoints(:,2),zSelectedPoints(:,3),'.');
        [row,~] = find(abs(Locks(i).pointCloud.Location(:,2) - xSaltatory(1,2)) < 8);
        ySelectedPoints = Locks(i).pointCloud.Location(row,:);
        hold on
        plot3(ySelectedPoints(:,1),ySelectedPoints(:,2),ySelectedPoints(:,3),'.');
        [c, ~, ~] = intersect(zSelectedPoints,ySelectedPoints,'rows');
        xminList = sortrows(c,1,'ascend');

        hold on
        plot3(xminList(:,1),xminList(:,2),xminList(:,3),'.');

        p3 = mean(xminList);
        [rightSideIndex,~] = find(xminList(:,1)<p3(1));
        xminList = xminList(rightSideIndex,:);
        p3 = mean(xminList);
        p3(2) = xSaltatory(2);


        hold on
        plot3(p3(1),p3(2),p3(3),'*g');
        [pcL,r] = points2circle(xSaltatory,p2,p3);
        delta = interval;
        %hold on
        %pcshow(pcL,'*');
        %hold on

        %第2次圆匹配
        h = pcL(3); % 高度
        % r = r;  %半径
        pos = [pcL(1),pcL(2)]; % 圆心位置
        t=0:0.001:(2*pi);  % 圆滑性设置
        t=[t,0];
        matchingCircle = [(pos(1)+r*sin(t))', (pos(2)+ones(size(t)))',(h+r*cos(t))'];


        matchingCircle = firstMatchingCircle;
        matchingCircle(:,1) = matchingCircle(:,1) - 1.1*r1;
        matchingCircle(:,3) =  matchingCircle(:,3) + 1.8*r1;






        wallCircle = matchingCircle;
        plot3(matchingCircle(:,1),matchingCircle(:,2),matchingCircle(:,3),'g.');
        hold on
        headFaceArc = sortrows(headFaceArc,1,'ascend');
        [row,~] = find(abs(headFaceArc(:,3) - Locks(i).panel) < 1);
        zpointsCandidate = headFaceArc(row,:);
        interPoint = sortrows(zpointsCandidate,1,'ascend');
        %panel和弧线交接点
        interPoint = interPoint(1,:);
        %hold on
        line1 = [Locks(i).xmin + toolDistance,Locks(i).ymin,interPoint(3),90,90,90;interPoint(1),Locks(i).ymin,interPoint(3),90,90,90];
        %plot3(line1(:,1),line1(:,2),line1(:,3),'*-w');
        [row,~] = find(headFaceArc(:,1)>interPoint(1) & headFaceArc(:,1)<xSaltatory(1,1));
        line2 = headFaceArc(row,:);
        line2(:,2) = Locks(i).ymin;

        row = size(line2,1);
        headFaceArcVector = zeros(row,3);
        for t=1:row
            headFaceArcVector(t,:) = [90, rad2deg(acos((pc(1)-line2(t,1))/r1)), 90];
        end

        line2 = [line2 headFaceArcVector];

        line2 = line2curve(line2);
        %         downSamplingIndex = 5;
        %         % %降采样
        %         if(downSamplingIndex>0)
        %             totalTime = downSamplingIndex;
        %             for time = 1: totalTime
        %                 evc = [2:2:size(line2,1)];
        %                 RESULT(evc,:) = [];
        %                 evc = [2:2:size(line2,1)];
        %                 line2(evc,:) = [];
        %             end
        %         end


        %
        %
        %         if i == 7
        %             jhi = 1;
        %         end
        %





        %hold on
        %plot3(line2(:,1),line2(:,2),line2(:,3),'*-g');
        [row,~] = find(wallCircle(:,1)>xSaltatory(1,1) & wallCircle(:,3) < pcL(3));
        wallCurveL = wallCircle(row,:);
        wallCurveL = sortrows(wallCurveL,1,'ascend');
        line3 = wallCurveL;
        line3(:,2) = Locks(i).ymin;
        line3(:,1) = line3(:,1) + (line2(end-1,1) - line3(1,1));
        line3(:,3) = line3(:,3) + (line2(end-1,3) - line3(1,3));

        row = size(line3,1);
        wallCurveLVector = zeros(row,3);
        for t=1:row
            wallCurveLVector(t,:) = [90, rad2deg(acos((line3(t,1) - pcL(1))/r)), 90];
        end

        line3 = [line3 wallCurveLVector];
        line3 = line2curve(line3);
        %         downSamplingIndex = 5;
        %         % %降采样
        %         if(downSamplingIndex>0)
        %             totalTime = downSamplingIndex;
        %             for time = 1: totalTime
        %                 evc = [2:2:size(line3,1)];
        %                 RESULT(evc,:) = [];
        %                 evc = [2:2:size(line3,1)];
        %                 line3(evc,:) = [];
        %             end
        %         end



        %hold on
        %plot3(line3(:,1),line3(:,2),line3(:,3),'*-w');


        line4 = [line3(end-1,1),line3(end-1,2),line3(end-1,3),90,0,90;line3(end-1,1),line3(end-1,2),Locks(i).zmax,90,0,90];



        %hold on
        %plot3(line4(:,1),line4(:,2),line4(:,3),'*-w');
        wallTrack = [line1;line2;line3;line4];
        delta = interval;
        allTimes = ceil((Locks(i).ymax - Locks(i).ymin) / delta) + 1;
        if mod((Locks(i).ymax - Locks(i).ymin), interval)< interval / 2
            allTimes = allTimes - 1;
        end



        for time=1:allTimes
            wallTracks(:,:,time) = [wallTrack(:,1),wallTrack(:,2) + delta - interval,wallTrack(:,3),wallTrack(:,4:6)];
            delta = delta + interval;
            %
            %

            plot3(wallTracks(:,1,time),wallTracks(:,2,time),wallTracks(:,3,time),'-*');
            hold on
        end
        endFaceArc = matchingCircle(headFaceArcRow,:);
        %endFaceArc(:,2) = Locks(i).ymax + toolDistance;
        %侧面墙轨迹第一次
        safetySpot = [];









        track7 = safetySpot;
        delta = interval;
        firstTimes = ceil((Locks(i).ymax - Locks(i).ymin) / 2 / delta);




        if Locks(i).ymax - Locks(i).ymin <= 1200
            firstTimes = allTimes;
            wallLineArea = 0;
            for time = 1:firstTimes

                %可以用非线性函数消除高度突变..
                wallTrackFit = wallTracks(:,:,time);
                ycurrent = wallTrackFit(10,2);
                [row,~] = find(abs(Locks(i).pointCloud.Location(:,2) - ycurrent) < 2);
                if size(row,1)==0
                    zcurrentMax = Locks(i).panel;
                else

                    pointtt = Locks(i).pointCloud.Location(row,:);
                    zcurrentMax = max(Locks(i).pointCloud.Location(row,3));
                end

                if wallTrackFit(end,3) >zcurrentMax

                    wallTrackFit(end,3) = zcurrentMax;

                end



                if wallTrackFit(end-3,3) > zcurrentMax

                    wallTrackFit(end-3,3) = zcurrentMax;
                    wallTrackFit(end-1,3) = zcurrentMax;
                    wallTrackFit(end,:) = [];
                end

                %             [row,~] = find(wallTrackFit(:,3) > zcurrentMax);
                %             wallTrackFit(row,:) = [];

                wallTrackFit = [ wallTrackFit;wallTrackFit(end,1),wallTrackFit(end,2),zcurrentMax - toolDistance + 90  + excess,wallTrackFit(end,4:6)];
                wallTrackFit = [ wallTrackFit;wallTrackFit(end,1)+100,wallTrackFit(end,2),zcurrentMax - toolDistance + 90 + excess,90,25,90];
                wallLineArea = wallLineArea + curveLen(wallTrackFit) * interval;

                %


                % [row,~] = find(wallTrackFit<-20000);
                %     wallTrackFit(row,:) = [];
                %

                %
                %                     hold on
                %                          plot3(wallTrackFit(:,1),wallTrackFit(:,2),wallTrackFit(:,3),'*-');
                if mod(time,2)
                    track7 = [track7;wallTrackFit(:,:)];
                else

                    %track7 = [track7;wallTrackFit(end:-1:1,:)];

                    for ii = size(wallTrackFit,1):-1:1
                        track7 = [track7;wallTrackFit(ii,:)];

                        if wallTrackFit(ii,1) == -80000
                            track7(end,:) = -150000;
                        end

                        if wallTrackFit(ii,1) == -150000
                            track7(end,:) = -80000;
                        end

                    end
                end
            end
            %
            %              [row,~] = find(track7(:,6) == 100);
            %              track7(row,:) = [];

            % [row,~] = find(track7<-20000);
            %     track7(row,:) = [];
            % plot3(track7(:,1),track7(:,2),track7(:,3),'*-');
            %

            track7 = [track7;safetySpot];
            %             hold on
            %             plot3(track7(:,1),track7(:,2),track7(:,3),'r');
            track7(:,1) = track7(:,1) - toolDistance;

            track7(:,3) = track7(:,3) + toolDistance;



        else

            wallLineArea = 0;
            for time = 1:firstTimes

                %可以用非线性函数消除高度突变..
                wallTrackFit = wallTracks(:,:,time);
                ycurrent = wallTrackFit(10,2);
                [row,~] = find(abs(Locks(i).pointCloud.Location(:,2) - ycurrent) < 2);
                if size(row,1)==0
                    zcurrentMax = Locks(i).panel;
                else

                    pointtt = Locks(i).pointCloud.Location(row,:);
                    zcurrentMax = max(Locks(i).pointCloud.Location(row,3));
                end

                if wallTrackFit(end,3) >zcurrentMax

                    wallTrackFit(end,3) = zcurrentMax;

                end



                if wallTrackFit(end-3,3) > zcurrentMax

                    wallTrackFit(end-3,3) = zcurrentMax;
                    wallTrackFit(end-1,3) = zcurrentMax;
                    wallTrackFit(end,:) = [];
                end

                %             [row,~] = find(wallTrackFit(:,3) > zcurrentMax);
                %             wallTrackFit(row,:) = [];

                wallTrackFit = [ wallTrackFit;wallTrackFit(end,1),wallTrackFit(end,2),zcurrentMax - toolDistance + 90  + excess,wallTrackFit(end,4:6)];
                wallTrackFit = [ wallTrackFit;wallTrackFit(end,1)+100,wallTrackFit(end,2),zcurrentMax - toolDistance + 90 + excess,90,60,90];
                wallLineArea = wallLineArea + curveLen(wallTrackFit) * interval;

                %


                % [row,~] = find(wallTrackFit<-20000);
                %     wallTrackFit(row,:) = [];
                %

                %
                %                     hold on
                %                          plot3(wallTrackFit(:,1),wallTrackFit(:,2),wallTrackFit(:,3),'*-');
                if mod(time,2)
                    track7 = [track7;wallTrackFit(:,:)];
                else

                    %track7 = [track7;wallTrackFit(end:-1:1,:)];

                    for ii = size(wallTrackFit,1):-1:1
                        track7 = [track7;wallTrackFit(ii,:)];

                        if wallTrackFit(ii,1) == -80000
                            track7(end,:) = -150000;
                        end

                        if wallTrackFit(ii,1) == -150000
                            track7(end,:) = -80000;
                        end

                    end
                end
            end
            %
            %              [row,~] = find(track7(:,6) == 100);
            %              track7(row,:) = [];

            % [row,~] = find(track7<-20000);
            %     track7(row,:) = [];
            % plot3(track7(:,1),track7(:,2),track7(:,3),'*-');
            %

            track7 = [track7;safetySpot];
            %             hold on
            %             plot3(track7(:,1),track7(:,2),track7(:,3),'r');




            %侧面墙轨迹第二次
            track8 = safetySpot;

            if Locks(i).ymax - Locks(i).ymin > 1200


                if mod(allTimes,2)
                    secondTimes = allTimes - firstTimes;
                else
                    secondTimes = allTimes - firstTimes + 1;
                end
                for time = allTimes:-1:secondTimes
                    wallTrackFit = wallTracks(:,:,time);
                    ycurrent = wallTrackFit(10,2);
                    [row,~] = find(abs(Locks(i).pointCloud.Location(:,2) - ycurrent) < 2);


                    if size(row,1) == 0

                        zcurrentMax = Locks(i). panel;
                    else
                        pointtt = Locks(i).pointCloud.Location(row,:);
                        zcurrentMax = max(Locks(i).pointCloud.Location(row,3));
                    end



                    %             [row,~] = find(wallTrackFit(:,3) > zcurrentMax);
                    %             wallTrackFit(row,:) = [];
                    if wallTrackFit(end,3) >zcurrentMax

                        wallTrackFit(end,3) = zcurrentMax;

                    end
                    if wallTrackFit(end-3,3)> zcurrentMax
                        wallTrackFit(end-3,3) = zcurrentMax;
                        wallTrackFit(end-1,3) = zcurrentMax;
                        wallTrackFit(end,:) = [];
                    end
                    wallTrackFit = [ wallTrackFit;wallTrackFit(end,1),wallTrackFit(end,2),zcurrentMax - toolDistance + 90 + excess,wallTrackFit(end,4:6)];
                    wallTrackFit = [ wallTrackFit;wallTrackFit(end,1)+100,wallTrackFit(end,2),zcurrentMax - toolDistance + 90 + excess,90,60,90];
                    %在此处完成脊面的变轨
                    wallLineArea = wallLineArea + curveLen(wallLineArea) * interval;

                    if mod(time,2)
                        % track8 = [track8;wallTrackFit(end:-1:1,:)];

                        for ii = size(wallTrackFit,1):-1:1
                            track8 = [track8;wallTrackFit(ii,:)];

                            if wallTrackFit(ii,1) == -80000
                                track8(end,:) = -150000;
                            end

                            if wallTrackFit(ii,1) == -150000
                                track8(end,:) = -80000;
                            end

                        end

                    else
                        track8 = [track8;wallTrackFit(:,:)];
                    end
                end
            end

            %         [row,~] = find(track8<-20000);
            %             track8(row,:) = [];
            %         plot3(track8(:,1),track8(:,2),track8(:,3),'*-');
            %
            %






            %         if track8(1,3)<track8(2,3)
            %             track8(1,:) = [];
            %         end

            %track4 = [track4;track3(size(track3,1)-1,:)];
            %track4 = [track4;safetySpot];
            %             hold on
            %             plot3(track4(:,1),track4(:,2),track4(:,3),'*-');


            track7(:,1) = track7(:,1) - toolDistance;
            track8(:,1) = track8(:,1) - toolDistance;

            track7(:,3) = track7(:,3) + toolDistance;
            track8(:,3) = track8(:,3) + toolDistance;

        end


        %前部端面
        track2 = safetySpot;

        row = size(headFaceArc,1);
        headFaceArcVector = zeros(row,6);
        for t=1:row
            headFaceArcVector(t,4:6) = [176,0,40];
        end
        headFaceArcVector(:,1:3) = sortrows(headFaceArc,1,'ascend');

        track2 = [track2;headFaceArcVector];
        track2 = [track2;safetySpot];
        [row,~] = find(track2(:,3)<track2(1,3)+40);
        track2(row,:) = [];
        track2 = line2curve(track2);

        %         downSamplingIndex = 6;
        %         %降采样
        %         if(downSamplingIndex>0)
        %             totalTime = downSamplingIndex;
        %             for time = 1: totalTime
        %                 evc = [2:2:size(track2,1)];
        %                 RESULT(evc,:) = [];
        %                 evc = [2:2:size(track2,1)];
        %                 track2(evc,:) = [];
        %             end
        %         end





        %             hold on
        %             plot3(track2(:,1),track2(:,2),track2(:,3),'*-');

        %hold on
        endFaceArc = headFaceArc;
        endFaceArc(:,2) = Locks(i).ymax + toolDistance;
        %plot3(endFaceArc(:,1),endFaceArc(:,2),endFaceArc(:,3),'*');

        %后部端面
        track5 = safetySpot;


        row = size(endFaceArc,1);
        endFaceArcVector = zeros(row,6);
        for t=1:row
            endFaceArcVector(t,4:6) = [5,4,-40];
        end
        endFaceArcVector(:,1:3) = sortrows(endFaceArc,1,'ascend');
        track5 = [track5;endFaceArcVector];
        track5 = [track5;safetySpot];
        [row,~] = find(track5(:,3)<track5(1,3)+40);
        track5(row,:) = [];
        track5 = line2curve(track5);


        %实际路线规划点位1 左侧立面
        delta = interval;
        times = ceil((Locks(i).panel - Locks(i).zmin) / delta);



        if Locks(i).ymax - Locks(i).ymin <= 1200

            track6 = safetySpot;
            for time = 1:times
                spot1 = [Locks(i).xmin-toolDistance, Locks(i).ymax + excess,Locks(i).zmin + delta-interval,96,0,0];
                spot2 = [ Locks(i).xmin-toolDistance, Locks(i).ymin - excess ,Locks(i).zmin + delta-interval,84,0,0];
                delta = delta+interval;
                if mod(time,2)
                    track6 = [track6;spot2;spot1];
                else
                    track6 = [track6;spot1;spot2];
                end
            end
            track6 = [track6;safetySpot];

        else



            track6 = safetySpot;
            for time = 1:times
                spot1 = [Locks(i).xmin-toolDistance, Locks(i).ymax + excess,Locks(i).zmin + delta-interval,96,0,0];
                spot2 = [ Locks(i).xmin-toolDistance, (Locks(i).ymin +Locks(i).ymax)/2 ,Locks(i).zmin + delta-interval,84,0,0];
                delta = delta+interval;
                if mod(time,2)
                    track6 = [track6;spot2;spot1];
                else
                    track6 = [track6;spot1;spot2];
                end
            end
            track6 = [track6;safetySpot];





            track9 = safetySpot;
            for time = 1:times
                spot1 = [Locks(i).xmin-toolDistance, Locks(i).ymax + excess,Locks(i).zmin + delta-interval,96,0,0];
                spot2 = [ Locks(i).xmin-toolDistance, (Locks(i).ymin +Locks(i).ymax)/2,Locks(i).zmin + delta-interval,84,0,0];
                delta = delta+interval;
                if mod(time,2)
                    track9 = [track9;spot2;spot1];
                else
                    track9 = [track9;spot1;spot2];
                end
            end
            track9 = [track9;safetySpot];


        end






        %右侧端面
        delta = interval;
        times = ceil((Locks(i).panel - Locks(i).zmin) / delta);


        if Locks(i).ymax - Locks(i).ymin <= 1200

            track1 = safetySpot;
            for time = 1:times
                spot1 = [Locks(i).xmax+toolDistance, Locks(i).ymax + excess,Locks(i).zmin + delta-interval,-84,0,0];
                spot2 = [ Locks(i).xmax+toolDistance, Locks(i).ymin - excess,Locks(i).zmin + delta-interval,-84,0,0];
                delta = delta+interval;
                if mod(time,2)
                    track1 = [track1;spot2;spot1];
                else
                    track1 = [track1;spot1;spot2];
                end
            end
            track1 = [track1;safetySpot];

        else


            track1 = safetySpot;
            for time = 1:times
                spot1 = [Locks(i).xmax+toolDistance, Locks(i).ymax + excess,Locks(i).zmin + delta-interval,-84,0,0];
                spot2 = [ Locks(i).xmax+toolDistance, (Locks(i).ymin+Locks(i).ymax)/2 ,Locks(i).zmin + delta-interval,-84,0,0];
                delta = delta+interval;
                if mod(time,2)
                    track1 = [track1;spot2;spot1];
                else
                    track1 = [track1;spot1;spot2];
                end
            end
            track1 = [track1;safetySpot];







            track10 = safetySpot;
            for time = 1:times
                spot1 = [Locks(i).xmax+toolDistance, Locks(i).ymin - excess,Locks(i).zmin + delta-interval,-84,0,0];
                spot2 = [ Locks(i).xmax+toolDistance, (Locks(i).ymin+Locks(i).ymax)/2,Locks(i).zmin + delta-interval,-84,0,0];
                delta = delta+interval;
                if mod(time,2)
                    track10 = [track10;spot2;spot1];
                else
                    track10 = [track10;spot1;spot2];
                end
            end
            track10 = [track10;safetySpot];






        end













        %             hold on
        %             plot3(track6(:,1),track6(:,2),track6(:,3),'*-');

        track3 = track7;
        track3(:,1) = - track3(:,1) + (Locks(i).xmax + Locks(i).xmin) ;
        track3(:,5) =  track3(:,5);
        track3(:,4) =  90;
        [row,~] = find(track3(:,5) > 177);

        track3(row,5) = track3(row,5) + 20000;

        track3(:,5) =  180 - track3(:,5);

        [row,~] = find( track3(:,5) < -10000 & track3(:,5) > -40000);

        track3(row,5) =  track3(row,5) + 20090;




        track3(:,6)  =  90;


        %             hold on
        %             plot3(track7(:,1),track7(:,2),track7(:,3),'*-');



        if Locks(i).ymax - Locks(i).ymin > 1200

            track4 = track8;

            track4(:,1) = -track4(:,1) + (Locks(i).xmax + Locks(i).xmin) ;


            [row,~] = find(track4(:,5) > 177);

            track4(row,5) = track4(row,5) + 20000;

            track4(:,5) =  180 - track4(:,5);


            [row,~] = find( track4(:,5) < -10000 & track4(:,5) > -40000);

            track4(row,5) =  track4(row,5) + 20000;

        end


        %         track2MinZ = min(track2(:,3));
        %         [row,~] = find(track2(:,3) < track2MinZ + 220);
        %         track2(row,:) = [];
        %
        %         track5MinZ = min(track5(:,3));
        %         [row,~] = find(track5(:,3) < track5MinZ + 220);
        %         track5(row,:) = [];


        %
        %         hold on;
        %         plot3(track1(:,1),track1(:,2),track1(:,3),'*-');hold on
        %         plot3(track2(:,1),track2(:,2),track2(:,3),'*-');hold on
        %         plot3(track3(:,1),track3(:,2),track3(:,3),'*-');hold on
        %         plot3(track4(:,1),track4(:,2),track4(:,3),'*-');hold on
        %         plot3(track5(:,1),track5(:,2),track5(:,3),'*-');hold on
        %         plot3(track6(:,1),track6(:,2),track6(:,3),'*-');hold on
        %         plot3(track7(:,1),track7(:,2),track7(:,3),'*-');hold on
        %         plot3(track8(:,1),track8(:,2),track8(:,3),'*-');hold on
        %
        %











        fileName =  [trackFolder,'Foundation-',int2str(i),'-','.txt'];
        paintingBase = Locks(i).paintingBase;
        save(fileName,'paintingBase','-ascii');
        %此处输出机器人指令
        %工件序号-点位序号-轨迹序号















        if Locks(i).ymax-Locks(i).ymin<=1200

            track1(:,3) =  620 - track1(:,3);
            track2(:,3) =  620 - track2(:,3);
            track3(:,3) =  620 - track3(:,3);
            track5(:,3) =  620 - track5(:,3);
            track6(:,3) =  620 - track6(:,3);
            track7(:,3) =  620 - track7(:,3);




            track1(:,1) = Locks(i).paintingBase(4,1) - track1(:,1);
            track1(:,2) = -Locks(i).paintingBase(4,2) + track1(:,2);
            track2(:,1) = Locks(i).paintingBase(6,1) - track2(:,1);
            track2(:,2) = -Locks(i).paintingBase(6,2) + track2(:,2);
            track3(:,1) = Locks(i).paintingBase(5,1) - track3(:,1);
            track3(:,2) = -Locks(i).paintingBase(5,2) + track3(:,2);
            track5(:,1) = Locks(i).paintingBase(3,1) - track5(:,1);
            track5(:,2) = -Locks(i).paintingBase(3,2) + track5(:,2);
            track6(:,1) = Locks(i).paintingBase(2,1) - track6(:,1);
            track6(:,2) = -Locks(i).paintingBase(2,2) + track6(:,2);
            track7(:,1) = Locks(i).paintingBase(1,1) - track7(:,1);
            track7(:,2) = -Locks(i).paintingBase(1,2) + track7(:,2);

            track1(:,[1;2]) = track1(:,[2;1]);
            track2(:,[1;2]) = track2(:,[2;1]);
            track3(:,[1;2]) = track3(:,[2;1]);

            track5(:,[1;2]) = track5(:,[2;1]);
            track6(:,[1;2]) = track6(:,[2;1]);
            track7(:,[1;2]) = track7(:,[2;1]);


            track2 = curveRemark(track2,1,0);
            track5 = curveRemark(track5,1,0);
            track3 = curveRemark(track3,1,0);
            track7 = curveRemark(track7,1,0);


            track5 = [track5(2,:);track5];
            track2 = [track2(2,:);track2];

            %
            %             hold on
            %             plot3(track1(:,1),track1(:,2),track1(:,3),'*-');hold on
            %             plot3(track2(:,1),track2(:,2),track2(:,3),'*-');hold on
            %             plot3(track3(:,1),track3(:,2),track3(:,3),'*-');hold on
            %             plot3(track4(:,1),track4(:,2),track4(:,3),'*-');hold on
            %             plot3(track5(:,1),track5(:,2),track5(:,3),'*-');hold on
            %             plot3(track6(:,1),track6(:,2),track6(:,3),'*-');hold on
            %             plot3(track7(:,1),track7(:,2),track7(:,3),'*-');hold on
            %             plot3(track8(:,1),track8(:,2),track8(:,3),'*-');hold on
            %
            %






            for spotNum = 1:6

                switch spotNum
                    case 1
                        fileName =  [trackFolder,int2str(i) ,'-','1.txt'];
                        save(fileName,'track7','-ascii');
                    case 2
                        fileName =  [trackFolder,int2str(i) ,'-','2.txt'];
                        save(fileName,'track6','-ascii');


                    case 3
                        fileName =  [trackFolder,int2str(i) ,'-','3.txt'];
                        save(fileName,'track5','-ascii');
                    case 4
                        fileName =  [trackFolder,int2str(i) ,'-','4.txt'];
                        save(fileName,'track1','-ascii');

                    case 5
                        fileName =  [trackFolder,int2str(i) ,'-','5.txt'];
                        save(fileName,'track3','-ascii');


                    case 6
                        fileName =  [trackFolder,int2str(i) ,'-','6.txt'];
                        save(fileName,'track2','-ascii');



                end

            end



        else



            track1(:,3) =  620 - track1(:,3);
            track2(:,3) =  620 - track2(:,3);
            track3(:,3) =  620 - track3(:,3);
            track4(:,3) =  620 - track4(:,3);
            track5(:,3) =  620 - track5(:,3);
            track6(:,3) =  620 - track6(:,3);
            track7(:,3) =  620 - track7(:,3);
            track8(:,3) =  620 - track8(:,3);
            track9(:,3) =  620 - track9(:,3);
            track10(:,3) =  620 - track10(:,3);




            track1(:,1) = Locks(i).paintingBase(6,1) - track1(:,1);
            track1(:,2) = -Locks(i).paintingBase(6,2) + track1(:,2);
            track2(:,1) = Locks(i).paintingBase(8,1) - track2(:,1);
            track2(:,2) = -Locks(i).paintingBase(8,2) + track2(:,2);
            track3(:,1) = Locks(i).paintingBase(5,1) - track3(:,1);
            track3(:,2) = -Locks(i).paintingBase(5,2) + track3(:,2);
            track4(:,1) = Locks(i).paintingBase(7,1) - track4(:,1);
            track4(:,2) = -Locks(i).paintingBase(7,2) + track4(:,2);
            track5(:,1) = Locks(i).paintingBase(4,1) - track5(:,1);
            track5(:,2) = -Locks(i).paintingBase(4,2) + track5(:,2);
            track6(:,1) = Locks(i).paintingBase(2,1) - track6(:,1);
            track6(:,2) = -Locks(i).paintingBase(2,2) + track6(:,2);
            track7(:,1) = Locks(i).paintingBase(1,1) - track7(:,1);
            track7(:,2) = -Locks(i).paintingBase(1,2) + track7(:,2);
            track8(:,1) = Locks(i).paintingBase(3,1) - track8(:,1);
            track8(:,2) = -Locks(i).paintingBase(3,2) + track8(:,2);
            track9(:,1) = Locks(i).paintingBase(9,1) - track9(:,1);
            track9(:,2) = -Locks(i).paintingBase(9,2) + track9(:,2);
            track10(:,1) = Locks(i).paintingBase(10,1) - track10(:,1);
            track10(:,2) = -Locks(i).paintingBase(10,2) + track10(:,2);

            track1(:,[1;2]) = track1(:,[2;1]);
            track2(:,[1;2]) = track2(:,[2;1]);
            track3(:,[1;2]) = track3(:,[2;1]);
            track4(:,[1;2]) = track4(:,[2;1]);
            track5(:,[1;2]) = track5(:,[2;1]);
            track6(:,[1;2]) = track6(:,[2;1]);
            track7(:,[1;2]) = track7(:,[2;1]);
            track8(:,[1;2]) = track8(:,[2;1]);
            track9(:,[1;2]) = track9(:,[2;1]);
            track10(:,[1;2]) = track10(:,[2;1]);

            track2 = curveRemark(track2,1,0);
            track5 = curveRemark(track5,1,0);

            track4 = curveRemark(track4,1,0);
            track3 = curveRemark(track3,1,0);

            track7 = curveRemark(track7,1,0);
            track8 = curveRemark(track8,1,0);



            track5 = [track5(2,:);track5];
            track2 = [track2(2,:);track2];


               

            

            %
            % [row,~] = find(track7<-20000);
            %     track7(row,:) = [];
            % plot3(track7(:,1),track7(:,2),track7(:,3),'*-');
            %




            for spotNum = 1:10

                switch spotNum
                    case 1
                        fileName =  [trackFolder,int2str(i) ,'-','1.txt'];
                        save(fileName,'track7','-ascii');
                    case 2
                        fileName =  [trackFolder,int2str(i) ,'-','-','2.txt'];
                        save(fileName,'track6','-ascii');


                    case 3
                        fileName =  [trackFolder,int2str(i) ,'-','-','3.txt'];
                        save(fileName,'track8','-ascii');
                    case 4
                        fileName =  [trackFolder,int2str(i) ,'-','4.txt'];
                        save(fileName,'track5','-ascii');

                    case 5
                        fileName =  [trackFolder,int2str(i) ,'-','5.txt'];
                        save(fileName,'track3','-ascii');


                    case 6
                        fileName =  [trackFolder,int2str(i) ,'-','6.txt'];
                        save(fileName,'track1','-ascii');

                    case 7
                        fileName =  [trackFolder,int2str(i) ,'-','7.txt'];
                        save(fileName,'track4','-ascii');


                    case 8
                        fileName =  [trackFolder,int2str(i) ,'-','8.txt'];
                        save(fileName,'track2','-ascii');

                    case 9
                        fileName =  [trackFolder,int2str(i),'-','9.txt'];
                        save(fileName,'track9','-ascii');


                    case 10
                        fileName =  [trackFolder,int2str(i) ,'-','10.txt'];
                        save(fileName,'track10','-ascii');
                end

            end

        end








        %面积计算(tentative,inaccurate)
        forehead = (interPoint(1) - Locks(i).xmin) * (Locks(i).panel - Locks(i).zmin);
        headFaceArea = forehead * 2;
        endFaceArea = headFaceArea;
        oneSide = (Locks(i).ymax - Locks(i).ymin) * (Locks(i).panel - Locks(i).zmin);
        sideArea = oneSide * 2;
        Locks(i).area = headFaceArea + endFaceArea + sideArea + wallLineArea * 2;

        fprintf("编号为%d的工件喷涂面积为%d \n",i,Locks(i).area);
        modelInfo(2,i+1) = Locks(i).area;

        clear wallTracks;
        massage = [int2str(i),"工件(下半件轨迹计算完成)"];
        disp(massage)
        toc;

    end











    fileName =  [trackFolder,int2str(casaNum),'-ProductMod','.txt'];

    locks4points(:,i) = [[Locks(i).xmin;Locks(i).ymin;Locks(i).panel];[Locks(i).xmin;Locks(i).ymax;Locks(i).panel];[Locks(i).xmax;Locks(i).ymin;Locks(i).panel];[Locks(i).xmax;Locks(i).ymax;Locks(i).panel]];

end

locks4points = [zeros(12,1),locks4points];

totalArea = 0;
for i = 1:size(Locks,2)
    if isempty(Locks(i).area)
    else
        totalArea = totalArea + Locks(i).area;
    end
end

modelInfo(2,1) = totalArea;



modelInfo = [modelInfo;locks4points];


save(fileName,'modelInfo','-ascii');


zip('1.zip','1');


end






