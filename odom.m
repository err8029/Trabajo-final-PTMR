function [pos_x, pos_y, pos_z, yaw] = odom()
    msg_odom = subscribe('odom');
    pose = msg_odom.Pose.Pose;
    position = pose.Position;
    orientation = pose.Orientation;
    %twist = msg_odom.Twist;
    
    [pos_x, pos_y, pos_z] = disp_position(position);
    [yaw] = disp_orientation(orientation);
end


function [x, y, z] = disp_position(position)
    x = position.X;
    y = position.Y;
    z = position.Z;
    
     disp('-------position------')
     disp(['x: ' num2str(position.X) ' m'])
     disp(['y: ' num2str(position.Y) ' m'])
     disp(['z: ' num2str(position.Z) ' m'])
end

function [yaw] = disp_orientation(orientation)
    q1=orientation.X;
    q2=orientation.Y;
    q3=orientation.Z;
    q4=orientation.W;
    
    [yaw, pitch, roll] = quaternions2deg(q1,q2,q3,q4);
    [yaw,quadrant]=angle_conv(yaw);
    
%     disp('-------orientation------')
     disp(['yaw (x): ' num2str(yaw) ' deg, quadrant: ' num2str(quadrant)])
%     disp(['pitch (y): ' num2str(pitch) ' deg'])
%     disp(['roll (z): ' num2str(roll) ' deg'])
end