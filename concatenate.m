function points = concatenate(PicQuantity,vehicleX,vehicleY,houseNum,imagePath,fastBoot,xRotate,yRotate,zRotate)
%需要合成的块数量
quantity = PicQuantity;
offset = 0;
src = zeros(1048576,3,quantity);RESULT = zeros(1048576*quantity,3);
casaNum = houseNum;
trackFolder = [int2str(casaNum),'\tracks\'];
[~] = mkdir(trackFolder);
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
        pcl(all(pcl==0,2),:) = -50000;
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
end
figure()
pcshow(ptCloud);
points = ptCloud;
end