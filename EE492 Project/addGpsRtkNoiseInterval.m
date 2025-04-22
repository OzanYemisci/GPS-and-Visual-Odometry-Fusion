function xyz_noisy = addGpsRtkNoiseInterval(xyz, sigma, probability)
% xyz: [N x 3] pozisyon verisi (x, y, z)
% sigma: [sig_x, sig_y] veya skaler (metre cinsinden)
% probability: 0 ile 1 arasında, gürültü ekleme olasılığı

    if isscalar(sigma)
        sigma = [sigma, sigma];
    end
    
    if nargin < 3
        probability = 1.0;  % Varsayılan: her aralıkta gürültü uygula
    end
    
    N = size(xyz, 1);
    noise = zeros(N, 2);

    % Gürültü ekleyeceğimiz aralıkları belirle
    noise_ranges = [20, 30; 70, 120; 300, 400; 500, 700];

    for j = 1:size(noise_ranges, 1)
        % Belirli bir olasılıkla gürültü ekle
        if rand() < probability
            startIdx = noise_ranges(j, 1);
            endIdx = noise_ranges(j, 2);
            
            if startIdx <= N
                endIdx = min(endIdx, N);
                noise(startIdx:endIdx, :) = randn(endIdx - startIdx + 1, 2) .* sigma;
            end
        end
    end

    xyz_noisy = xyz;
    xyz_noisy(:, 1:2) = xyz(:, 1:2) + noise;  % sadece x ve y'ye gürültü ekle
end
