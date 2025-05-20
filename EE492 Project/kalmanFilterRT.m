function xyz_smoothed = kalmanFilterRT(xyz, timestamps)
    % Giriş: xyz = [Nx3] ölçüm verisi (x, y, z)
    %         timestamps = datetime array of measurement times
    % Çıkış: EKF ile filtrelenmiş veri (xyz_smoothed)
    
    N = size(xyz, 1);
    xyz_smoothed = zeros(N, 3);

    stateTransition = @(state, dt) [state(1) + dt*state(3); 
                                   state(2) + dt*state(4); 
                                   state(3); 
                                   state(4)];
    
    measurementFcn = @(state) [state(1); state(2)];
    
    % EKF nesnesini oluştur
    ekfObj = extendedKalmanFilter(stateTransition, measurementFcn, [xyz(1,1); xyz(1,2); 0; 0]);
    
    % Gürültü kovaryansları (per second)
    ekfObj.ProcessNoise = 0.01 * eye(4);
    ekfObj.MeasurementNoise = 0.7 * eye(2);

    prev_time = timestamps(1);
    for i = 1:N
        current_time = timestamps(i);
        dt = seconds(current_time - prev_time);
        prev_time = current_time;
        
        % **Tahmin Aşaması (Predict) with actual dt**
        predict(ekfObj, dt);
        
        % **Ölçüm Güncellemesi (Update)**
        z = xyz(i, 1:2)';
        correct(ekfObj, z);
        
        xyz_smoothed(i, 1:2) = ekfObj.State(1:2);
        xyz_smoothed(i, 3) = xyz(i, 3);
    end
end