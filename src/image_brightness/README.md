Package image_brightness
-----------------------------------------------
Description:
Node that subscribes to a camera image topic and publishes a boolean
indicating whether the average brightness of the image is above a
configurable threshold. Brightness is determined by converting the
image to grayscale and computing the mean pixel value (0-255).

Output:
/light_state 
        Type: std_msgs/msg/Bool
        Binary value indicating whether the average luminosity of the image is over the set threshold.

Input:
/image
        Type: sensor_msgs/msg/Image
        Published image by the cam2image_vm2ros.

Parameter:
brightness_threshold
        Type: double (default: 64.0, range: 0.0-255.0)
        If the mean grayscale pixel value of the image exceeds this value, /light_state is
        published as true.

Get the dependencies:
    rosdep install -i --from-path src --rosdistro jazzy -y

Build:
    colcon build --packages-select image_brightness

Source the setup file (new terminal, ws root):
    . install/setup.bash

Run the node:
    ros2 run image_brightness image_brightness_node
Run with a custom threshold:
    ros2 run image_brightness image_brightness_node --ros-args -p brightness_threshold:=100.0
Update threshold at runtime:
    ros2 param set /image_brightness_node brightness_threshold 124.0

Monitor output:
    ros2 topic echo /light_state