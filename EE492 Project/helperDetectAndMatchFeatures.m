function [currPoints, currFeatures, indexPairs] = helperDetectAndMatchFeatures(prevFeatures, Irgb, yoloxModel)
    numPoints = 200;

    % Gri tonlamaya çevir
    Igray = rgb2gray(Irgb);

    % YOLOX ile nesne tespiti yap
    [bboxes, scores, labels] = detect(yoloxModel, Irgb); % Orijinal renkli görüntüyü kullan

    % Sadece güven skoru yüksek olan nesneleri seç
    minScore = 0.9;
    validIdx = scores > minScore;
    bboxes = bboxes(validIdx, :);

    % Maske oluştur (başlangıçta tüm pikseller false, yani tüm alanlar kullanılabilir)
    mask = false(size(Igray));

    % Eğer nesne tespit edilmişse, bounding box alanlarını maskeye ekle
    if ~isempty(bboxes)
        for i = 1:size(bboxes, 1)
            x = max(1, round(bboxes(i, 1)));  % X başlangıç noktası
            y = max(1, round(bboxes(i, 2)));  % Y başlangıç noktası
            w = round(bboxes(i, 3));          % Genişlik
            h = round(bboxes(i, 4));          % Yükseklik
            
            % Görüntü sınırlarını aşmamak için kontrol et
            xEnd = min(size(Igray,2), x+w);
            yEnd = min(size(Igray,1), y+h);

            % Maskeyi güncelle (hareketli nesnelerin olduğu bölgeleri işaretle)
            mask(y:yEnd, x:xEnd) = true;
        end
    end

    % SURF feature'larını tespit et
    allPoints = detectSURFFeatures(Igray, 'MetricThreshold', 500);

    % Sadece statik alanlarda kalan noktaları seç
    validIdx = ~mask(sub2ind(size(mask), round(allPoints.Location(:,2)), round(allPoints.Location(:,1))));
    currPoints = allPoints(validIdx);

    % Belirli sayıda feature noktası seç (eğer yeterli nokta yoksa fallback yap)
    if length(currPoints) > numPoints
        currPoints = selectUniform(currPoints, numPoints, size(Igray));
    end

    % Seçilen noktaların özelliklerini çıkar
    currFeatures = extractFeatures(Igray, currPoints, 'Upright', true);

    % Önceki karedeki feature'larla eşleştirme yap
    indexPairs = matchFeatures(prevFeatures, currFeatures, 'Unique', true, 'MaxRatio', 0.9);
end
