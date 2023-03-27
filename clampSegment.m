
function Locks = clampSegment(ptCloud)

minDistance = 50;
minPoints = 9000;
[labels,numClusters] = pcsegdist(ptCloud,minDistance,'NumClusterPoints',minPoints);

for i=1:numClusters
    idxValidPoints = find(labels == i);
    segmentedPtCloud = select(ptCloud,idxValidPoints);

    figure
    pcshow(segmentedPtCloud);

    Locks(i) = LockClamp(i,segmentedPtCloud);
end
disp("点云分类和切割完成")
fprintf("本批次工件共有%d个",numClusters);

end
