# ROS2 URDF 패키지 실행 가이드

이 문서는 `robot-link-forge`에서 내보낸 URDF ROS2 패키지를 실행하는 방법을 안내합니다.

## ROS2 (`colcon build`) 에서 실행하기

1.  **ROS2 워크스페이스 준비**:
    ROS2 워크스페이스가 없다면 새로 생성합니다.

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/
    colcon build
    source install/setup.bash
    ```

2.  **패키지 압축 해제**:
    다운로드한 `<robot_name>_ros_package.zip` 파일을 `src` 디렉토리로 이동하고 압축을 해제합니다.

    ```bash
    mv /path/to/<robot_name>_ros_package.zip ~/ros2_ws/src/
    cd ~/ros2_ws/src/
    unzip <robot_name>_ros_package.zip
    rm <robot_name>_ros_package.zip
    ```

3.  **빌드**:
    워크스페이스의 최상위 디렉토리로 돌아와서 `colcon build`를 실행하여 패키지를 빌드합니다.

    ```bash
    cd ~/ros2_ws/
    colcon build --symlink-install
    ```

4.  **환경 설정**:
    새로운 터미널을 열거나 기존 터미널에서 워크스페이스의 `setup.bash` 파일을 소싱합니다.

    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

5.  **RViz 실행**:
    `ros2 launch` 명령어를 사용하여 `display.launch.py` 파일을 실행합니다. 그러면 RViz가 실행되고 로봇 모델이 나타납니다. `joint_state_publisher_gui`를 사용하여 각 조인트를 움직여볼 수 있습니다.

    ```bash
    ros2 launch <robot_name> display.launch.py
    ```
    (`<robot_name>`은 내보낸 로봇의 이름으로 변경해야 합니다.)
