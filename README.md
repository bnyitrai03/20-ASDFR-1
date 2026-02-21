Get the dependencies:
    rosdep install -i --from-path src --rosdistro jazzy -y

Build all of the packages:
    colcon build

Build specific packages:
    colcon build --packages-select ...

Source the setup files (new terminal, ws root):
    . install/setup.bash

Run the nodes:
    ros2 run ...