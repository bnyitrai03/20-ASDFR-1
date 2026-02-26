# Package image_processing
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
    ```
    rosdep install -i --from-path src --rosdistro jazzy -y
    ```

## Build:
    ```
    colcon build --packages-select image_processing
    ```

## Source the setup file (new terminal, ws root):
    ```
    . install/setup.bash
    ```

## Run the node:
    ```
    ros2 run image_processing image_processing_node
    ```

## Update params at runtime:
    ```
    ros2 param set /image_processing_node brightness_threshold 124.0
    ```

## Monitor output:
    ```
    ros2 topic echo /object_coordinate
    ```