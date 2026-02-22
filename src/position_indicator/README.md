# Package position_indicator
-----------------------------------------------
## Description:
    ROS2 node that detects the position of a bright object in an
    image using grayscale thresholding and CoG computation.
    If no object is found (no pixels above threshold),
    coordinates (0, 0) are published with z = 0 to signal no detection.

## Output:
/object_coordinate 
        Type: geometry_msgs/msg/Point
        x: horizontal pixel coordinate (column) of the detected object centre
        y: vertical pixel coordinate (row) of the detected object centre
        z: detection flag — 1.0 if object detected, 0.0 if not found

## Input:
/image
        Type: sensor_msgs/msg/Image
        Input image stream.

## Parameters:
brightness_threshold
        Type: double (default: 64.0, range: 0.0-255.0)
        Pixels with grayscale intensity above this value are treated 
        as the object.

image_topic
        Type: std_string (default: /image)
        Topic to subscribe to for the camera image stream.

## Get the dependencies:
    ```bash
    rosdep install -i --from-path src --rosdistro jazzy -y
    ```

## Build:
    ```bash
    colcon build --packages-select position_indicator
    ```

## Source the setup file (new terminal, ws root):
    ```bash
    . install/setup.bash
    ```

## Run the node:
    ```bash
    ros2 run position_indicator position_indicator_node
    ```

## Update params at runtime:
    ```bash
    ros2 param set /position_indicator_node brightness_threshold 124.0
    ```

## Monitor output:
    ```bash
    ros2 topic echo /object_coordinate
    ```