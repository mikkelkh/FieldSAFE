# Usage
Ground truth data for dynamic obstacles is contained in *dynamic_ground_truth.txt*.
It has the following format:

    1   Track ID. All rows with the same ID belong to the same path.
    2   x. The x-coordinate (column) of the object in vatic.
    3   y. The y-coordinate (row) of the object in vatic.
    4   frame. The vatic frame that this annotation represents.
    5   timestamp. The corresponding ROS timestamp.
    6   lost. If 1, the annotation is outside of the view screen.
    7   occluded. Unused, ignore.
    8   generated. If 1, the annotation was automatically interpolated.
    9   label. The label for this annotation, enclosed in quotation marks.
    10  state. The state of the object if any.

A simple usage example is shown in *demo.m*, where the dynamic obstacles (humans) and the tractor are plotted over time on top of the annotated static map.