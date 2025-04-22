function xyz_noisy = addGpsRtkNoise(xyz, sigma, probability)
% xyz: [N x 3] pozisyon verisi (x, y, z)
% sigma: [sig_x, sig_y] veya skaler (metre cinsinden)
% probability: her noktaya gürültü ekleme olasılığı (0-1), default = 1.0

    if nargin < 3
        probability = 1.0;
    end
    if isscalar(sigma)
        sigma = [sigma, sigma];
    end

    N = size(xyz, 1);
    noise = zeros(N, 2);

    for i = 1:N
        if rand() < probability
            noise(i, :) = randn(1, 2) .* sigma;
        end
    end

    xyz_noisy = xyz;
    xyz_noisy(:, 1:2) = xyz(:, 1:2) + noise;  % sadece x ve y'ye ekle
end