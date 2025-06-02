function [vSet, temp_camPoses] = helperNormalizeViewSet(vSet, xyz)
camPoses = poses(vSet);
numViews = height(camPoses);

% Define the range of views to consider
maxViews = numViews;
minViews = max(2, numViews - 10);

% Extract relevant portion of ground truth locations
locationsGT = xyz(minViews:maxViews, :);

% Move the first camera to the origin
locations = vertcat(camPoses.AbsolutePose.Translation);
locations = locations - locations(1, :);
locations = locations(minViews:maxViews, :);

% Compute scale factor using only X and Y axes
magnitudesXY = mean(vecnorm(diff(locations(:, [3, 1])), 2, 2)); % (X, -Y)
magnitudesGTXY = mean(vecnorm(diff(locationsGT(:, [1, 2])), 2, 2)); % (X, Y)
scaleFactor = min(magnitudesGTXY / magnitudesXY, 10); % Limit scaling to avoid extreme values

% Rotation matrix to align first camera along Z-axis
R = camPoses.AbsolutePose(1).R;
R_inv = R'; % Precompute inverse rotation

% Apply transformations to camera poses
temp_camPoses = camPoses;
for i = minViews:maxViews
    % Scale and update translations
    deltaTranslation = camPoses.AbsolutePose(i).Translation - camPoses.AbsolutePose(i-1).Translation;
    temp_camPoses.AbsolutePose(i).Translation = temp_camPoses.AbsolutePose(i-1).Translation + deltaTranslation * scaleFactor*0.5;
    
    % Rotate poses to align with the Z-axis-
    temp_camPoses.AbsolutePose(i).R = R_inv * camPoses.AbsolutePose(i).R;
end

% Update view set with new camera poses
vSet = updateView(vSet, temp_camPoses);
end

