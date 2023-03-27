function restoredTrack = restoreTrack(track)
[row,~] = find(track(:,1) == -80000 | track(:,1) == -150000);

track(row,:) = [];

restoredTrack = track;

end