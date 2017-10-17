clear all;close all;

% Load annotated map
fileImage = 'static_ground_truth.png';
imgMap = imread(fileImage);

% Load example GPS coordinates in UTM
% In this example, we load the GPS coordinates of the drone markers
A = dlmread('gps_marker_positions.csv', ';', 1, 0);
kml_point_number = A(:,1);
img_col = A(:,2);
img_row = A(:,3);
utm_x = A(:,4);
utm_y = A(:,5);
altitude = A(:,6);

% Construct homogeneous UTM coordinates
UTM = [utm_x';utm_y';ones(1,length(utm_x))];

% Load transformation matrix from UTM coordinates to pixel coordinates in
% annotated map
transform = dlmread('utm2PixelsTransformMatrix.csv', ',', 0, 0);

% Apply transform on UTM coordinates
img_coords = transform*UTM
img_coords = img_coords./repmat(img_coords(3,:),3,1);

% Visualize
% Transformed UTM coordinates are marked with red crosses on top of the
% drone makers (yellow regions in annotated map)
imshow(imgMap)
hold on;
plot(img_coords(2,:),img_coords(1,:),'rx')