clear all;
close all;


%工艺参数等设置
fastBoot = 1;
PicQuantity = 20;
vehicleX = [600, 1850, 3100, 4350, 5600, 5600, 4350, 3100, 1850, 600, 600, 1850, 3100, 4350, 5600, 5600, 4350, 3100, 1850, 600];
vehicleY = [0, 0, 0, 0, 0, 1500, 1500, 1500, 1500, 1500, 2500, 2500, 2500, 2500, 2500, 3500, 3500, 3500, 3500, 3500];
xRotate = 5.0;
yRotate = -3.0;
zRotate = -1.8;
houseNum = 1;
toolDistance = 190;
excess = 100;
imagePath = 'Point';




%合成20张点云图片
points = concatenate(PicQuantity,vehicleX,vehicleY,houseNum,imagePath,fastBoot,xRotate,yRotate,zRotate);


%点云分割
% Locks = clampSegment(points);




whole = pcread("wholePC.ply");
Locks = clampSegment(whole);


radius = 290;
deltaZ = 223;


close all
figure
%计算大于1200的上半件
for i = 1:size(Locks,2)

if Locks(i).type == "上半件" && Locks(i).lengthType == "short"
[radius,deltaZ] = topLessThan1200(Locks(i));
end
% if Locks(i).type == "下半件" && Locks(i).lengthType == "short"
% bottomMoreThan1200(Locks(i), radius,deltaZ);
% end




if Locks(i).type == "上半件" && Locks(i).lengthType == "long"
[radius,deltaZ] = topMoreThan1200(Locks(i));
end
if Locks(i).type == "下半件" && Locks(i).lengthType == "long"
bottomMoreThan1200(Locks(i), radius,deltaZ);
end





end


















