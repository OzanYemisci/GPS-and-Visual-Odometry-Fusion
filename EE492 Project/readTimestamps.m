function timestamps = readTimestamps(filename)
    % KITTI timestamp formatını doğru şekilde okuyan fonksiyon
    fid = fopen(filename, 'r');
    if fid == -1
        error('Dosya bulunamadı: %s', filename);
    end
    
    % Tüm satırları oku
    lines = textscan(fid, '%s', 'Delimiter', '\n');
    fclose(fid);
    lines = lines{1};
    
    % Timestamp'leri parse et
    timestamps = datetime(lines, 'InputFormat', 'yyyy-MM-dd HH:mm:ss.SSSSSSSSS');
end