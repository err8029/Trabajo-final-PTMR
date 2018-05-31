function laserData = readLaserData(laserMsg)
%readPose Extract the laser data in Cartesian coordinates
laserData = readCartesian(laserMsg) * [0 1; -1 0];
end
