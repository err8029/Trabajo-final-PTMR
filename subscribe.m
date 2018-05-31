function msg = subscribe(topic)
    sub = rossubscriber(topic);
    msg = sub.LatestMessage;
    while isempty(msg)
        msg = sub.LatestMessage;
    end
end

