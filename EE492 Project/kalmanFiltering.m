function filtered_positions = kalmanFiltering(noisy_xyz, vo_positions, rotations)
    % Parametreler
    dt = 1/10;
    Q = 0.15 * eye(4);          % Süreç gürültüsü
    R_gps_nominal = 0.05 * eye(2);      % Temel GPS gürültüsü
    R_vo_nominal = 0.05 * eye(2); % Temel VO gürültüsü
    
    % Rotasyon büyüklüklerini hesapla
    rotation_magnitudes = zeros(length(rotations), 1);
    for k = 2:length(rotations)
        if ~isempty(rotations{k}) && ~isempty(rotations{k-1})
            R_prev = rotations{k-1};
            R_curr = rotations{k};
            R_inc = R_curr * R_prev';
            theta = acos((trace(R_inc) - 1)/2);
            rotation_magnitudes(k) = theta / dt;
        end
    end
    
    % 2. Aritmetik ortalama uygula (5-frame penceresi)
    window_size = 20;
    smoothed_rot_mags = zeros(size(rotation_magnitudes));
    for k = 1:length(rotation_magnitudes)
        start_idx = max(1, k - window_size + 1);
        smoothed_rot_mags(k) = mean(rotation_magnitudes(start_idx:k));
    end
    
    % 3. Normalizasyon (0-1 arası)
    rotation_magnitudes = smoothed_rot_mags / max(smoothed_rot_mags); 
   
    % UKF başlat
    ukf = trackingUKF(...
        'StateTransitionFcn', @(state, dt) [state(1)+dt*state(3); state(2)+dt*state(4); state(3); state(4)], ...
        'MeasurementFcn', @(state) [state(1); state(2)], ...
        'State', [noisy_xyz(1,1); noisy_xyz(1,2); 0; 0], ...
        'ProcessNoise', Q);
    
    % Geçmiş ölçümleri'ları saklamak için
    window_size = 5;
    innov_gps_history = zeros(2, window_size);
    innov_vo_history = zeros(2, window_size);
    
    % Filtreleme
    filtered_positions = zeros(size(noisy_xyz, 1), 2);
    for k = 1:size(noisy_xyz, 1)
        % --- PREDICT ---
        predict(ukf, dt);
        
        % --- ÖLÇÜMLER ---
        z_gps = [noisy_xyz(k,1); noisy_xyz(k,2)];
        z_vo = [vo_positions(k,1); vo_positions(k,2)];
        
        % --- Measurement Residual hesaplama ---
        innov_gps = z_gps - ukf.MeasurementFcn(ukf.State);
        innov_vo = z_vo - ukf.MeasurementFcn(ukf.State);
        
        % Innovation geçmişini güncelle
        innov_gps_history = [innov_gps_history(:,2:end), innov_gps];
        innov_vo_history = [innov_vo_history(:,2:end), innov_vo];
        
        % --- ADAPTIVE NOISE AYARI ---
        % 1. Rotasyon faktörü (VO için)
        rot_factor = 1 + rotation_magnitudes(k)/5; % 1x-3x
        
        % 2. VO Measurement Residual faktörü (son 5 frame'in ortalaması)
        vo_innov_factor = 1 + 0.3 * mean(vecnorm(innov_vo_history, 2, 1));
        
        % 3. GPS Measurement Residual faktörü
        gps_innov_factor = 1 + 0.3 * mean(vecnorm(innov_gps_history, 2, 1));
        
        % Gürültü kovaryanslarını güncelle
        R_vo = R_vo_nominal *rot_factor * vo_innov_factor;
        R_gps = R_gps_nominal;
        
        % --- CORRECT ---
        % 1. GPS düzeltme (adaptif gürültü)
        ukf.MeasurementNoise = R_gps;
        correct(ukf, z_gps);
        
        % 2. VO düzeltme (adaptif gürültü)
        ukf.MeasurementNoise = R_vo;
        correct(ukf, z_vo);
        
        filtered_positions(k, :) = ukf.State(1:2)';
        fprintf('Frame %d: R_gps=%.4f, R_vo=%.4f\n', k, R_gps(1,1), R_vo(1,1));

    end
end

