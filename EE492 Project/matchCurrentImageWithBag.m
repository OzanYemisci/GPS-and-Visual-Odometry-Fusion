function [best_match_pos, best_match_id] = matchCurrentImageWithBag(bag, currentImage, intrinsics)
    % currentImage: Mevcut işlenen görüntü (RGB veya gri)
    % intrinsics: Kamera parametreleri (undistort için)
    
    % 1. Mevcut görüntüyü işle (undistort + feature extraction)
    currentI = undistortImage(im2gray(currentImage), intrinsics);
    currentPoints = detectSURFFeatures(currentI, 'MetricThreshold', 500);
    [currentFeatures, ~] = extractFeatures(currentI, currentPoints, 'Upright', true);
    
    % 2. Bag'deki tüm görüntülerle karşılaştır
    numImages = numel(bag);
    matchScores = zeros(numImages, 1); % Her görüntü için skor tut
    
    for i = 1:numImages
        % Bag'deki feature'ları al
        bagFeatures = bag(i).Feature;
        bagPoints = bag(i).Points;
        
        % Feature matching yap (Brute-Force veya FLANN)
        indexPairs = matchFeatures(bagFeatures, currentFeatures, ...
                                  'MatchThreshold', 50, ...
                                  'MaxRatio', 0.7, ...
                                  'Unique', true);
        
        % Eşleşme skorunu hesapla (ne kadar çok eşleşme varsa o kadar iyi)
        matchScores(i) = size(indexPairs, 1);
    end
    
    % 3. En yüksek skora sahip görüntüyü bul
    [~, best_match_id] = max(matchScores);
    best_match_pos = bag(best_match_id).Positions;
    
    fprintf('En iyi eşleşme: Görüntü %d, Konum: [%.2f, %.2f, %.2f]\n', ...
            best_match_id, best_match_pos(1), best_match_pos(2), best_match_pos(3));
end