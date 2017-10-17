clear all; close all;

% Load dynamic ground truth data
table = readtable('dynamic_ground_truth.txt','ReadVariableNames',true,'Delimiter',',');

% Load tractor path in GPS coordinates
tableTractor = readtable('tractor_gps_coordinates.csv','ReadVariableNames',true,'Delimiter',',');

% Load transformation between GPS coordinates and pixel coordinates
transform = dlmread('../static/utm2PixelsTransformMatrix.csv', ',', 0, 0);

% Load static map for overlaying positions
imgMap = imread('../static/static_ground_truth.png');

% Visualize
figure(1)
imshow(imgMap);

% Plotting options
axImg = gca;
ax1 = axes('position',axImg.Position,'Ydir','reverse');
set(ax1, 'Color', 'none','TickDir','out')
xlim([0 size(imgMap,2)]);
ylim([0 size(imgMap,1)]);

% Run through all annotated frames
for frame=min(table.frame):5:max(table.frame)
    delete(allchild(ax1))
    hold on;
    
    % Get corresponding frame information
    mask = table.frame == frame;
    id = table.track_id(mask);
    x = table.x(mask);
    y = table.y(mask);
    label = table.label(mask);
    state = table.state(mask);
    lost = table.lost(mask);
    occluded = table.occluded(mask);
    timestamp = unique(table.timestamp(mask));

    %  Show the dynamic obstacles (x'es in figure)
    plotInds = not(occluded)&not(lost)&strcmp(label,'human');
    plot(x(plotInds),y(plotInds),'rx');
    
    % Get corresponding tractor location
    [~, clockIdx] = min(abs(tableTractor.clock - timestamp));
    [utm_x,utm_y,utmzone] = deg2utm(tableTractor.lat(clockIdx), tableTractor.lon(clockIdx));
    tractor_xy = transform * [utm_x;utm_y;1];
    
    % Show the tractor path (* in figure)
    plot(tractor_xy(2),tractor_xy(1),'b*');
    
    % Plotting options
    set(ax1, 'Color', 'none')
    xlim([0 size(imgMap,2)]);
    ylim([0 size(imgMap,1)]);
    
    if frame==min(table.frame)
        legend('Dynamic obstacles (humans)','Tractor')
    end
    
    drawnow;
end