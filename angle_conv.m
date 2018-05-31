
function [angle,q] = angle_conv(angle_ans)
    if abs(angle_ans)>90
        if angle_ans<0
            angle = -(-angle_ans - 180);
            q=3;
        elseif angle_ans>0
            angle = angle_ans - 90;
            q=2;
        end
    elseif abs(angle_ans)<90
        if angle_ans<0
            angle = -(-angle_ans-90);
            q=4;
        elseif angle_ans>0
            angle = angle_ans;
            q=1;
        end
    end
end