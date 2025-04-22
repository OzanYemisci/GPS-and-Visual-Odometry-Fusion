function xyz_smoothed = kalmanFilterRT(xyz)
    % Giriş: xyz = [Nx3] ölçüm verisi (x, y, z)
    % Çıkış: EKF ile filtrelenmiş veri (xyz_smoothed)
    
    N = size(xyz, 1);
    xyz_smoothed = zeros(N, 3);

    % **Durum Modeli**: x = [x, y, vx, vy]
    dt = 0.1;  % Örnekleme periyodu
    f = @(x) [x(1) + dt*x(3); x(2) + dt*x(4); x(3); x(4)];  % Doğrusal olmayan geçiş modeli
    h = @(x) [x(1); x(2)];  % Ölçüm modeli (sadece x ve y okunuyor)

    % EKF nesnesini oluştur
    ekfObj = extendedKalmanFilter(f, h, [xyz(1,1); xyz(1,2); 0; 0]); % Başlangıç durumu

    % Gürültü kovaryansları (optimize edilebilir)
    ekfObj.ProcessNoise = 0.01 * eye(4);  % Süreç gürültüsü
    ekfObj.MeasurementNoise = 0.7 * eye(2); % Ölçüm gürültüsü

    for i = 1:N
        % **Tahmin Aşaması (Predict)**
        predict(ekfObj);
        
        % **Ölçüm Güncellemesi (Update)**
        z = xyz(i, 1:2)'; % Ölçüm verisi
        correct(ekfObj, z);
        
        % Güncellenmiş tahmini kaydet
        xyz_smoothed(i, 1:2) = ekfObj.State(1:2);
        xyz_smoothed(i, 3) = xyz(i, 3); % Z ekseni değiştirilmeden korunuyor
    end
end