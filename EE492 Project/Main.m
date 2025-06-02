clc; clear all;
addpath('devkit/matlab/');
oxts = loadOxtsliteData('2011_09_26_drive_0022_sync');
pose = convertOxtsToPose(oxts);
rng(1);
selectedClasses = ["car", "person", "bicycle", "motorcycle", "bus", "truck"];
yoloxModel = yoloxObjectDetector("tiny-coco", selectedClasses);

gps_timestamps = readTimestamps('2011_09_26_drive_0022_sync/oxts/timestamps.txt');
image_timestamps = readTimestamps('2011_09_26_drive_0022_sync/image_02/timestamps.txt');
global_base_time = min([gps_timestamps(1); image_timestamps(1)]);
% 4. Normalizasyon yap
[gps_sec, ~] = normalizeTimestamps(gps_timestamps, global_base_time);
[img_sec, ~] = normalizeTimestamps(image_timestamps, global_base_time);

%% 1. Veri Hazırlığı
% GPS ve poz verilerini yükle
xyz = zeros(length(pose), 3);
for viewId = 1:length(pose)
    if ~isempty(pose{viewId})
        xyz(viewId, :) = pose{viewId}(1:3, 4)'; 
    end
end
noisy_xyz = addGpsRtkNoise(xyz,3,1);
xyz_smoothed = kalmanFilterRT(noisy_xyz,gps_timestamps);



% Kamera parametreleri
calibration = loadCalibrationCamToCam('calib_cam_to_cam.txt');
P_matrix = calibration.P_rect{1};
focalLength = [P_matrix(1,1) P_matrix(2,2)];
principalPoint = [P_matrix(1,3) P_matrix(2,3)];
images = imageDatastore('2011_09_26_drive_0022_sync/image_02/data');
Irgb = readimage(images, 1); 
imageSize = size(Irgb,[1,2]);
intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

%% 4. Ana İşlem Döngüsü
% İlk görüntüyü işleme
prevI = undistortImage(im2gray(Irgb), intrinsics); 

% Detect features. 
prevPoints = detectSURFFeatures(prevI, MetricThreshold=500);

% Select a subset of features, uniformly distributed throughout the image.
numPoints = 200;
prevPoints = selectUniform(prevPoints, numPoints, size(prevI));

% Extract features. Using 'Upright' features improves matching quality if 
% the camera motion involves little or no in-plane rotation.
prevFeatures = extractFeatures(prevI, prevPoints, Upright=true);

vSet = imageviewset;

% Add the first view. Place the camera associated with the first view
% at the origin, oriented along the Z-axis.
viewId = 1;
vSet = addView(vSet, viewId, rigidtform3d(eye(3), [0 0 0]), Points=prevPoints);

% Convert to gray scale and undistort.
viewId = 2;
Irgb = readimage(images, viewId);
I = undistortImage(im2gray(Irgb), intrinsics);

% Match features between the previous and the current image.
[currPoints, currFeatures, indexPairs] = helperDetectAndMatchFeatures(...
    prevFeatures, Irgb, yoloxModel);

% Estimate the pose of the current view relative to the previous view.
[relPose, inlierIdx] = helperEstimateRelativePose(...
    prevPoints(indexPairs(:,1)), currPoints(indexPairs(:,2)), intrinsics);

% Exclude epipolar outliers.
indexPairs = indexPairs(inlierIdx, :);

% Add the current view to the view set.
vSet = addView(vSet, viewId, relPose, Points=currPoints);

% Store the point matches between the previous and the current views.
vSet = addConnection(vSet, viewId-1, viewId, Matches=indexPairs);
prevI = I;
prevFeatures = currFeatures;
prevPoints   = currPoints;


for viewId = 3:15
    Irgb = readimage(images, viewId);
    % Read and undistort the current image
    I = undistortImage(im2gray(Irgb), intrinsics);

    % Match features between previous and current image
    [currPoints, currFeatures, indexPairs] = helperDetectAndMatchFeatures(prevFeatures, Irgb, yoloxModel);

    % Estimate the relative pose of the current view
    [relPose, inlierIdx] = helperEstimateRelativePose(prevPoints(indexPairs(:,1)), currPoints(indexPairs(:,2)), intrinsics);
    indexPairs = indexPairs(inlierIdx, :);
    
    % Triangulate points from the previous two views, and find the 
    % corresponding points in the current view.
    [worldPoints, imagePoints] = helperFind3Dto2DCorrespondences(vSet,...
        intrinsics, indexPairs, currPoints);
    
    warningstate = warning('off','vision:ransac:maxTrialsReached');
    
    
    % Estimate the world camera pose for the current view.
    absPose = estworldpose(imagePoints, worldPoints, intrinsics);
    
    % Restore the original warning state
    warning(warningstate)
    
    % Add the current view to the view set.
    vSet = addView(vSet, viewId, absPose, Points=currPoints);
    
    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, viewId-1, viewId, Matches=indexPairs);    
    
    tracks = findTracks(vSet); % Find point tracks spanning multiple views.
        
    camPoses = poses(vSet);    % Get camera poses for all views.
    
    % Triangulate initial locations for the 3-D world points.
    xyzPoints = triangulateMultiview(tracks, camPoses, intrinsics);
    
    % Refine camera poses using bundle adjustment.
    [~, camPoses] = bundleAdjustment(xyzPoints, tracks, camPoses, ...
        intrinsics, PointsUndistorted=true, AbsoluteTolerance=1e-7,...
        RelativeTolerance=1e-15, MaxIterations=20, FixedViewID=1, Solver="preconditioned-conjugate-gradient");
        
    vSet = updateView(vSet, camPoses); % Update view set.
   
    % Bundle adjustment can move the entire set of cameras. Normalize the
    % view set to place the first camera at the origin looking along the
    % Z-axes and adjust the scale to match that of the ground truth.
    vSet = helperNormalizeViewSet(vSet, xyz_smoothed);
    
    prevI = I;
    prevFeatures = currFeatures;
    prevPoints   = currPoints;  
end

%%
for viewId = 16:numel(images.Files)
    % Read and display the next image
    Irgb = readimage(images, viewId);
    
    % Convert to gray scale and undistort.
    I = undistortImage(im2gray(Irgb), intrinsics);

    % Match points between the previous and the current image.
    [currPoints, currFeatures, indexPairs] = helperDetectAndMatchFeatures(...
        prevFeatures, Irgb, yoloxModel);    
          
    % Triangulate points from the previous two views, and find the 
    % corresponding points in the current view.
    [worldPoints, imagePoints] = helperFind3Dto2DCorrespondences(vSet, ...
        intrinsics, indexPairs, currPoints);

    % Since RANSAC involves a stochastic process, it may sometimes not
    % reach the desired confidence level and exceed maximum number of
    % trials. Disable the warning when that happens since the outcomes are
    % still valid.
    warningstate = warning('off','vision:ransac:maxTrialsReached');
    
    % Estimate the world camera pose for the current view.
    absPose = estworldpose(imagePoints, worldPoints, intrinsics);
    
    % Restore the original warning state
    warning(warningstate)
    
    % Add the current view and connection to the view set.
    vSet = addView(vSet, viewId, absPose, Points=currPoints);
    vSet = addConnection(vSet, viewId-1, viewId, Matches=indexPairs);
        
    % Refine estimated camera poses using windowed bundle adjustment. Run 
    % the optimization every 30th view.
    time1 = datetime('now');
    if mod(viewId, 30) == 0 || viewId == height(noisy_xyz)  
        windowSize = 30;
        camPoses = poses(vSet);
        
        vSetLast15 = takeLast15ViewSet(vSet, camPoses, windowSize);
        minimum = min(viewId-1,31);
        
        startFrame = max(1, viewId - windowSize);
        tracks = findTracks(vSetLast15, MinTrackLength=5);
        camPoses = poses(vSetLast15);
        [xyzPoints, reprojErrors] = triangulateMultiview(tracks, camPoses, intrinsics);
                                
        fixedIds = [startFrame, startFrame+1];
        idx = reprojErrors < 1.5;
        
        [~, camPoses] = bundleAdjustment(xyzPoints(idx, :), tracks(idx), ...
            camPoses, intrinsics, FixedViewIDs=fixedIds, ...
            PointsUndistorted=true, AbsoluteTolerance=1e-12,...
            RelativeTolerance=1e-12, MaxIterations=200);
    
        vSet = updateView(vSet, camPoses);
        [vSet, temp_camPoses] = helperNormalizeViewSet(vSet, xyz_smoothed);
    

        matched_pos = xyz_smoothed(viewId, : );

        % 1. Mevcut VO pozisyonunu al
        current_vo_pose = temp_camPoses.AbsolutePose(end);
        current_vo_pos = current_vo_pose.Translation;

        % Transform to VO coordinate system
        target_pos  = [-matched_pos(2), matched_pos(3),  matched_pos(1)];
        
        translation_vector = target_pos - current_vo_pos;

        soft_translation = translation_vector * 1;
        start_idx = max(2, height(temp_camPoses)-30); % Son 30 kare (veya daha az varsa)

        for i = start_idx:height(temp_camPoses)
            weight = min(1,(i+1 - start_idx)/30);
            temp_camPoses.AbsolutePose(i).Translation = ...
                temp_camPoses.AbsolutePose(i).Translation + translation_vector * weight;
        end

    
        % Update view set with the fused pose
        vSet = updateView(vSet, temp_camPoses);
    end
    time2 = datetime('now');
    timeDifference = between(time1, time2);
    disp(viewId +" time "+ char(timeDifference));

    
    prevI = I;
    prevFeatures = currFeatures;
    prevPoints   = currPoints;  
end
%%

camPoses = poses(vSet); % Kamera pozlarını al
positions = vertcat(camPoses.AbsolutePose.Translation);
positions = positions(:, [3, 1, 2]); % (X, Y, Z) yerine (-Y, Z, X)
positions(:,2) = -positions(:,2);    % Y eksenini ters çevir


figure;
hold on;
plot3(positions(:,1), positions(:,2), positions(:,3), '-', 'LineWidth', 2); % Kamera trajesi (mavi)
plot3(xyz(:,1), xyz(:,2), xyz(:,3), '-', 'LineWidth', 2); % Gerçek pozisyonlar (kırmızı)
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
legend('Visual Odometry Trajectory', 'Ground Truth Trajectory');
title('Camera Trajectory Comparison: VO vs Ground Truth');
hold off;



%%
% Kalman Filtresi
rotations = cell(height(camPoses), 1);
for viewId = 1:height(camPoses)
    rotations{viewId} = camPoses.AbsolutePose(viewId).R; % 3x3 rotasyon matrisi
end

filtered_positions = kalmanFiltering(xyz_smoothed, positions, rotations, gps_timestamps, image_timestamps); 
filtered_positions = filtered_positions(1:2:end, :);
% Sonuçları Görselleştirme
figure;
hold on;
plot(positions(:,1), positions(:,2), '-', 'LineWidth', 2); % VO Trajesi
plot(xyz(:,1), xyz(:,2), '-', 'LineWidth', 2); % GPS Trajesi
plot(filtered_positions(:,1), filtered_positions(:,2), '-', 'LineWidth', 2); % Filtrelenmiş Traje
plot(xyz_smoothed(:,1), xyz_smoothed(:,2),  '-', 'LineWidth', 2, 'Color', 'b'); % Noisy GPS Trajesi
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
legend('VO Trajectory', 'Ground Truth', 'UKF(GPS+VO Fusion) Estimated Trajectory','Kalman-Smoothed GPS');
title('Trajectory Comparison: UKF Estimation (GPS + VO)');
hold off;

figure;
hold on;
plot(noisy_xyz(:,1), noisy_xyz(:,2),  '.', 'LineWidth', 2, 'Color', 'm'); % Noisy GPS Trajesi
plot(xyz_smoothed(:,1), xyz_smoothed(:,2),  '.', 'LineWidth', 2, 'Color', 'b'); % Noisy GPS Trajesi
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
legend('Raw Noisy GPS','Kalman-Smoothed GPS');
title('GPS Data Comparison: Raw vs Smoothed');
hold off;
% XY düzlemindeki filtrelenmiş pozisyon hatası
positionErrorXY = vecnorm(filtered_positions(:, 1:2) - xyz(:, 1:2), 2, 2); % Filtrelenmiş ve ground truth

% XY düzlemindeki noisy_xyz pozisyon hatası
noisyErrorXY = vecnorm(noisy_xyz(:, 1:2) - xyz(:, 1:2), 2, 2); % gps ve ground truth
% XY düzlemindeki noisy_xyz pozisyon hatası
smoothErrorXY = vecnorm(xyz_smoothed(:, 1:2) - xyz(:, 1:2), 2, 2); % smoothXYgps ve ground truth

% Yüzdelik iyileşmeyi hesapla

improvementPercentage = ((smoothErrorXY - positionErrorXY) ./ smoothErrorXY) * 100; % Yüzde iyileşme
improvementPercentage(improvementPercentage < 0) = 0; % Negatif değerleri sıfır yap
% Grafiklerin çizimi
figure;
% noisy_GPS pozisyon hatası
subplot(3, 1, 1);
plot(gps_timestamps,noisyErrorXY, '-', 'LineWidth', 2, 'Color', 'r');
xlabel('Time (s)');
ylabel('Position Error (m)');
title('Position Error: Raw Noisy GPS vs Ground Truth');
grid on;
xlim([min(gps_timestamps), max(gps_timestamps)]); 

% smoothed_noisy_gps pozisyon hatası
subplot(3, 1, 2);
plot(gps_timestamps,smoothErrorXY, '-', 'LineWidth', 2, 'Color', 'b');
hold on;
plot(gps_timestamps,positionErrorXY, '-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Position Error (m)');
title('Position Error Comparison: Kalman-Smoothed GPS vs UKF(GPS+VO Fusion) Estimation');
grid on;
legend('Kalman-Smoothed GPS Error','UKF(GPS+VO Fusion) Estimated Error');
xlim([min(gps_timestamps), max(gps_timestamps)]); 

subplot(3, 1, 3);
plot(gps_timestamps,improvementPercentage, '-', 'LineWidth', 2, 'Color', 'k');
xlabel('Time (s)');
ylabel('Improvement (%)');
title('Error Reduction: UKF(GPS+VO Fusion) vs Kalman-Smoothed GPS');
grid on;
xlim([min(gps_timestamps), max(gps_timestamps)]); 

% positionErrorXY için istatistiksel değerler
meanPositionError = mean(positionErrorXY);
maxPositionError = max(positionErrorXY);
minPositionError = min(positionErrorXY);
fprintf('Position Error (UKF): Mean = %.2f m, Max = %.2f m, Min = %.2f m\n', ...
    meanPositionError, maxPositionError, minPositionError);

% noisyErrorXY için istatistiksel değerler
meanNoisyError = mean(noisyErrorXY);
maxNoisyError = max(noisyErrorXY);
minNoisyError = min(noisyErrorXY);
fprintf('Noisy GPS Error: Mean = %.2f m, Max = %.2f m, Min = %.2f m\n', ...
    meanNoisyError, maxNoisyError, minNoisyError);

% smoothErrorXY için istatistiksel değerler
meanSmoothError = mean(smoothErrorXY);
maxSmoothError = max(smoothErrorXY);
minSmoothError = min(smoothErrorXY);
fprintf('Kalman-Smoothed GPS Error: Mean = %.2f m, Max = %.2f m, Min = %.2f m\n', ...
    meanSmoothError, maxSmoothError, minSmoothError);

% improvementPercentage için istatistiksel değerler
meanImprovement = mean(improvementPercentage);
maxImprovement = max(improvementPercentage);
minImprovement = min(improvementPercentage);
fprintf('Improvement Percentage: Mean = %.2f%%, Max = %.2f%%, Min = %.2f%%\n', ...
    meanImprovement, maxImprovement, minImprovement);