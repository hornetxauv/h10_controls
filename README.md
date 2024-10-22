# Setup

Always remember to run `source /opt/ros/humble/setup.bash`

## File Structure
The files on this repo should be placed in the SRC folder of your hornet workspace
E.g.
 - HornetXAuV_ws
    - src
        - controls_movement
        - launch_files

## Steps to get started

1. Use `mkdir` to make your own src folder
2. git pull the stuff into there
3. `cd ..` and then from HornetXAuv_ws run
4. `colcon build`
5. `source install/setup.bash`
6. `ros2 run controls_movement moveLeft`
7. Alternatively, (under construction) run `ros 2 launch launch_files pool_test.launch.py`

After completing the above steps your file structure will look like

 - HornetXAuV_ws
    - build
    - install
    - log
    - src
        - controls_movement
        - launch_files
