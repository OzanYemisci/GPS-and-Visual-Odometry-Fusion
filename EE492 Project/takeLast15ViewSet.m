function vSetLast15 = takeLast15ViewSet(vSet, camPoses, windowSize)
    % Son 15 görüntünün ViewId'lerini al
    numViews = numel(camPoses.ViewId);
    last15ViewIds = camPoses.ViewId(max(1, numViews-windowSize):end);
    
    % Yeni bir vSetLast15 nesnesi oluştur
    vSetLast15 = imageviewset;
    
    % Son 15 görüntüyü vSetLast15'e ekle
    for i = 1:numel(last15ViewIds)
        viewId = last15ViewIds(i);
        pose = camPoses.AbsolutePose(viewId == camPoses.ViewId);
        points = vSet.Views.Points{viewId == vSet.Views.ViewId}; % Feature points
        vSetLast15 = addView(vSetLast15, viewId, pose, Points=points);
    end
    
    % Son 15 görüntü arasındaki bağlantıları ekle
    connections = vSet.Connections;
    for i = 1:height(connections)
        if ismember(connections.ViewId1(i), last15ViewIds) && ...
           ismember(connections.ViewId2(i), last15ViewIds)
            vSetLast15 = addConnection(vSetLast15, ...
                connections.ViewId1(i), connections.ViewId2(i), ...
                Matches=connections.Matches{i});
        end
    end
    
    disp("Son 15 görüntü yeni vSetLast15 nesnesine kaydedildi.");
end
