function [timestamps_sec, base_time] = normalizeTimestamps(timestamps_datetime, base_time)
    % İsteğe bağlı base_time alır veya ilk timestamp'i baz alır
    if nargin < 2 || isempty(base_time)
        base_time = timestamps_datetime(1);
    end
    
    timestamps_sec = seconds(timestamps_datetime - base_time);
    
    % Validasyon
    fprintf('Zaman aralığı: %.3f saniye\n', timestamps_sec(end));
    fprintf('Örnekleme sıklığı: ~%.1f Hz\n', 1/mean(diff(timestamps_sec)));
end