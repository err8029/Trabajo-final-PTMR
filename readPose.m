function pose = readPose(odomMsg,robot)
%readPose Extract the robot odometry reading as [x y theta] vector

% Extract the x, y, and theta coordinates
poseMsg = odomMsg.Pose.Pose;
xpos = poseMsg.Position.X+robot(:,1);
ypos = poseMsg.Position.Y+robot(:,2);
quat = poseMsg.Orientation;
angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
theta = angles(1);
disp(theta);
pose = [xpos, ypos, theta];
end