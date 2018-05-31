function [yaw,pitch,roll] =  quaternions2deg(q1,q2,q3,q4)
    sinr = 2*(q4*q1 + q2*q3);
    cosr = 1 - 2*(q1*q1 + q2*q2);
    roll = atan2(sinr,cosr);
    roll = radtodeg(roll);
    
    sinp = 2*(q4 * q2 - q3 * q1);
    if (abs(sinp) >=1)
        pitch = copysign(pi/2, sinp);
    else
        pitch = asin(sinp);
    end
    pitch = radtodeg(pitch);
    
    siny = 2*(q4*q3 + q1*q2);
    cosy = 1 - 2*(q2*q2 + q3*q3);
    yaw = atan2(siny,cosy);
    yaw = radtodeg(yaw);
    
end