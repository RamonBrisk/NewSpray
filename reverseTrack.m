function reversedTrack = reverseTrack(track)

reversedTrack = flipud(track);

for i=1:size(reversedTrack,1)

    if reversedTrack(i) == -80000
        reversedTrack(i) = -150000;

    elseif reversedTrack(i) == -150000
        reversedTrack(i) = -80000;

    end


end
end