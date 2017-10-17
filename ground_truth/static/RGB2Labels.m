clear all; close all;

fileImage = 'static_ground_truth.png';
img = imread(fileImage);

% Read labels.csv
tabLabels = readtable('labels.csv','ReadVariableNames',1,'Delimiter',',');
tabLabels.Properties.RowNames = tabLabels.Label;

% Construct colormap for converting between RGB colors and label indices
map = [tabLabels.R,tabLabels.G,tabLabels.B]./255;

% Convert to grayscale image with label indices (ranges from 0 to size(tabLabels,1)-1)
imgLabels = uint8(rgb2ind(double(img)./255,map));

imwrite(uint8(imgLabels),'static_ground_truth_indices.png', 'BitDepth', 8);

% Here, label remapping to new classes can be performed, if desired
remapping = false;
if remapping
    remap = [1:size(tabLabels,1)];

    % For instance, the following line replaces all instances of
    % 'road' with 'ground'.
    remap(tabLabels.ID('road')) = tabLabels.ID('ground')

    % Construct labeled image with remapped classes
    imgLabelsRemapped = zeros(size(imgLabels));
    for r=1:length(remap)
        imgLabelsRemapped(imgLabels==r-1) = remap(r)-1;
    end

    % Write remapped, labeled image
    imwrite(uint8(imgLabelsRemapped),'static_ground_truth_indices_remapped.png', 'BitDepth', 8)
end