## Report:
https://www.overleaf.com/project/698c707341eb6048a47ea37d

## Get the dependencies:
    ```bash
    rosdep install -i --from-path src --rosdistro jazzy -y
    ```

## Build all of the packages:
    ```bash
    colcon build
    ```

## Build specific packages:
    ```bash
    colcon build --packages-select ...
    ```

## Source the setup files (new terminal, ws root):
    ```bash
    . install/setup.bash
    ```

## Run the nodes:
    ```bash
    ros2 run ...
    ```

## Use the launch file:
    ```
    cd launch
    ros2 launch relbot_sim_launch.yaml
    ```
