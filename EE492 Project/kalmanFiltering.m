function filtered_positions = kalmanFiltering(noisy_xyz, vo_positions, rotations, gps_timestamps, vo_timestamps)
    % Parametreler
    Q = 0.05 * eye(4);                  % Süreç gürültüsü
    R_gps_nominal = 0.3 * eye(2);      % Temel GPS gürültüsü
    R_vo_nominal = 0.1 * eye(2);       % Temel VO gürültüsü

    % UKF başlat
    ukf = trackingUKF(...
        'StateTransitionFcn', @(state, dt) [state(1)+dt*state(3); state(2)+dt*state(4); state(3); state(4)], ...
        'MeasurementFcn', @(state) [state(1); state(2)], ...
        'State', [noisy_xyz(1,1); noisy_xyz(1,2); 0; 0], ...
        'ProcessNoise', Q);

    % Geçmiş innovation’ları tutmak için
    window_size = 5;
    innov_gps_history = zeros(2, window_size);
    innov_vo_history = zeros(2, window_size);

    % Tüm timestamp’leri sırala
    all_times = [gps_timestamps; vo_timestamps];
    all_labels = [repmat("gps", size(gps_timestamps)); repmat("vo", size(vo_timestamps))];
    [sorted_times, sort_idx] = sort(all_times);
    sorted_labels = all_labels(sort_idx);

    % İndeks tutucular
    gps_idx = 1;
    vo_idx = 1;


    % Sonuçları depolamak için
    filtered_positions = [];

    prev_time = sorted_times(1);
    for k = 1:length(sorted_times)
        current_time = sorted_times(k);
        dt = seconds(current_time - prev_time);
        prev_time = current_time;

        % --- PREDICT ---
        predict(ukf, dt);

        if sorted_labels(k) == "gps"
            % --- GPS Ölçüm ve güncelleme ---
            z_gps = [noisy_xyz(gps_idx,1); noisy_xyz(gps_idx,2)];
            innov_gps = z_gps - ukf.MeasurementFcn(ukf.State);
            innov_gps_history = [innov_gps_history(:,2:end), innov_gps];
            gps_innov_factor = 1 + 0.3 * mean(vecnorm(innov_gps_history, 2, 1));
            R_gps = R_gps_nominal * gps_innov_factor;
            ukf.MeasurementNoise = R_gps;
            correct(ukf, z_gps);
            gps_idx = gps_idx + 1;

        elseif sorted_labels(k) == "vo"
            % --- VO Ölçüm ve güncelleme ---
            z_vo = [vo_positions(vo_idx,1); vo_positions(vo_idx,2)];
            innov_vo = z_vo - ukf.MeasurementFcn(ukf.State);
            innov_vo_history = [innov_vo_history(:,2:end), innov_vo];
            vo_innov_factor = 1 + 0.3 * mean(vecnorm(innov_vo_history, 2, 1));
        
            % Rotation faktörü anlık hesapla
            rot_factor = 1;
            if vo_idx > 1 && ~isempty(rotations{vo_idx}) && ~isempty(rotations{vo_idx-1})
                R_prev = rotations{vo_idx-1};
                R_curr = rotations{vo_idx};
                R_inc = R_curr * R_prev';
                theta = acos((trace(R_inc) - 1)/2);
                dt_rot = seconds(vo_timestamps(vo_idx) - vo_timestamps(vo_idx - 1));
                if dt_rot > 0
                    rot_mag = theta / dt_rot;
                    rot_factor = 1 + rot_mag / 5;
                else
                    rot_factor = 1;  % veya önceki değer kullanılabilir
                end
            end
        
            R_vo = R_vo_nominal * rot_factor * vo_innov_factor;
            ukf.MeasurementNoise = R_vo;
            correct(ukf, z_vo);
            vo_idx = vo_idx + 1;
        end


        % Son pozisyonu kaydet
        filtered_positions(end+1, :) = ukf.State(1:2)';
        fprintf('Step %d: dt=%.3fs, label=%s\n', k, dt, sorted_labels(k));
    end
end
