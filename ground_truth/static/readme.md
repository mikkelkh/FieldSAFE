# Usage
Ground truth data for static obstacles is contained in *static_labels.png*.

The conversion from RGB-values to class labels is available in *labels.csv*.

A number of GPS markers were placed on the field and measured with exact GPS positions.
There are available in *gps_marker_positions.csv* along with the corresponding pixel coordinates (for looking up in *static_labels.png*).
The correspondences have been used to generate a transformation matrix that converts UTM coordinates to pixel coordinates (x and y). Both have to be specified in homogeneous coordinates.
The transformation matrix is available in *utm2PixelsTransformMatrix.csv*. A simple usage example is shown in *demo.m*.

The script *RGB2Labels.m* converts from the RGB-colored labels in *static_labels.png* to label indices.
