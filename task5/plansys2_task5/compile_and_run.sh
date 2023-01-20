source /opt/ros/humble/setup.bash
rosdep install --from-paths ./ --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch plansys2_task5 plansys2_task5_launch.py
