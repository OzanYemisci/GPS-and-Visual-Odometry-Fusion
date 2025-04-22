% helperTriangulateLastFrames triangulate points from last two views
%  [worldPoints, imagePoints] = helperTriangulateLastFrames(vSet, 
%  cameraParams, matchIdx, currPoints) returns world points triangulated 
%  from the last two views in the view set, which are also visible in the 
%  current image.
%
%  vSet is a viewSet object, cameraParams is a cameraParameters object, and
%  matchIdx is an M-by-2 matrix of matched indices between the points in
%  the last view of vSet and the current image.
%
%  worldPoints is a three-column matrix containing the [x,y,z] coordinates
%  of world points which are visible in the last two views of vSet and the
%  current image. idxTriplet is a vector containing the indices of points
%  in the current image corresponding to worldPoints.
%
%  See also triangulate, imageviewset, estworldpose.

% Copyright 2016-2022 The MathWorks, Inc. 
function [worldPoints, imagePoints] = helperFind3Dto2DCorrespondences(vSet, ...
    intrinsics, matchIdx, currPoints)

camPoses = poses(vSet);

% Compute the camera projection matrix for the next-to-the-last view.
camMatrix1 = cameraProjection(intrinsics, pose2extr(camPoses.AbsolutePose(end-1)));

% Compute the camera projection matrix for the last view.
camMatrix2 = cameraProjection(intrinsics, pose2extr(camPoses.AbsolutePose(end)));

% Find indices of points visible in all three views.
matchIdxPrev = vSet.Connections.Matches{end};
[~, ia, ib] = intersect(matchIdxPrev(:, 2), matchIdx(:, 1));
idx1 = matchIdxPrev(ia, 1);
idx2 = matchIdxPrev(ia, 2);
idxTriplet = matchIdx(ib, 2);

% Triangulate the points.
points1 = vSet.Views.Points{end-1};
points2 = vSet.Views.Points{end};
worldPoints = triangulate(points1(idx1,:),... 
    points2(idx2,:), camMatrix1, camMatrix2);
imagePoints = currPoints(idxTriplet).Location;


