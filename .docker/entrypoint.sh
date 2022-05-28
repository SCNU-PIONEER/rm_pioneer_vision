#!/bin/zsh

case $ROBOT in
    hero)
        export ROS_DOMAIN_ID=1
        camera_type=mindvision
        ;;
    standard3)
        export ROS_DOMAIN_ID=3
        camera_type=hik
        ;;
    standard4)
        export ROS_DOMAIN_ID=4
        camera_type=hik
        ;;
    standard5)
        export ROS_DOMAIN_ID=5
        camera_type=mindvision
        ;;
    guard_top)
        export ROS_DOMAIN_ID=6
        camera_type=mindvision
        ;;
    guard_bottom)
        export ROS_DOMAIN_ID=7
        camera_type=realsense
        ;;
    *)
        echo "Unknown robot type: $ROBOT"
        ;;
esac

source /opt/ros/galactic/setup.zsh

case $camera_type in
    mindvision)
        if [ ! "$(ls -A /root/ros_ws/src/ros2_mindvision_camera)" ]; then
            git clone https://github.com/chenjunnn/ros2_mindvision_camera.git src/ros2_mindvision_camera
            colcon build --symlink-install --packages-select mindvision_camera
        fi
        ;;
    hik)
        if [ ! "$(ls -A /root/ros_ws/src/ros2_hik_camera)" ]; then
            export MVCAM_SDK_PATH=/root/ros_ws/src/ros2_hik_camera/hikSDK
            export MVCAM_COMMON_RUNENV=/root/ros_ws/src/ros2_hik_camera/hikSDK/lib
            export LD_LIBRARY_PATH=/root/ros_ws/src/ros2_hik_camera/hikSDK/lib:$LD_LIBRARY_PATH

            git clone https://github.com/nolem-77/ros2_hik_camera.git src/ros2_hik_camera
            colcon build --symlink-install --packages-select hik_camera

            echo "export MVCAM_SDK_PATH=/root/ros_ws/src/ros2_hik_camera/hikSDK" >> /root/.zshrc
            echo "export MVCAM_COMMON_RUNENV=/root/ros_ws/src/ros2_hik_camera/hikSDK/lib" >> /root/.zshrc
            echo "export LD_LIBRARY_PATH=/root/ros_ws/src/ros2_hik_camera/hikSDK/lib:$LD_LIBRARY_PATH" >> /root/.zshrc
        fi
        ;;
    *)
        echo "Unknown camera type: $camera_type"
        ;;
esac

# Clone config repo if it doesn't exist
if [ ! "$(ls -A /root/ros_ws/src/rm_pioneer_config)" ]; then
    git clone https://github.com/chenjunnn/rm_pioneer_config -b ${ROBOT} src/rm_pioneer_config
fi

colcon build --symlink-install --packages-select rm_pioneer_description rm_pioneer_bringup

exec "$@"
