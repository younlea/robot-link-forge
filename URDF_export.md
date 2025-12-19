# URDF 패키지 실행 가이드

이 문서는 `robot-link-forge`에서 내보낸 URDF ROS 패키지를 실행하는 방법을 안내합니다.

현재 내보내진 패키지는 **ROS1** (`catkin_make`)을 기준으로 생성되었습니다.

## ROS1 (`catkin_make`) 에서 실행하기

1.  **ROS 워크스페이스 준비**:
    ROS `catkin` 워크스페이스가 없다면 새로 생성합니다.

    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make
    source devel/setup.bash
    ```

2.  **패키지 압축 해제**:
    다운로드한 `<robot_name>_ros_package.zip` 파일을 `src` 디렉토리로 이동하고 압축을 해제합니다.

    ```bash
    mv /path/to/<robot_name>_ros_package.zip ~/catkin_ws/src/
    cd ~/catkin_ws/src/
    unzip <robot_name>_ros_package.zip
    rm <robot_name>_ros_package.zip
    ```

3.  **빌드**:
    워크스페이스의 최상위 디렉토리로 돌아와서 `catkin_make`를 실행하여 패키지를 빌드합니다.

    ```bash
    cd ~/catkin_ws/
    catkin_make
    ```

4.  **환경 설정**:
    새로운 터미널을 열거나 기존 터미널에서 워크스페이스의 `setup.bash` 파일을 소싱합니다.

    ```bash
    source ~/catkin_ws/devel/setup.bash
    ```

5.  **RViz 실행**:
    `roslaunch` 명령어를 사용하여 `display.launch` 파일을 실행합니다. 그러면 RViz가 실행되고 로봇 모델이 나타납니다. `joint_state_publisher_gui`를 사용하여 각 조인트를 움직여볼 수 있습니다.

    ```bash
    roslaunch <robot_name> display.launch
    ```
    (`<robot_name>`은 내보낸 로봇의 이름으로 변경해야 합니다.)

---

## ROS2 (`colcon build`) 에서 실행하기 (참고)

현재 생성된 패키지는 ROS1용이지만, ROS2에서 사용하려면 약간의 수정이 필요합니다.

1.  **`package.xml` 수정**: `<buildtool_depend>catkin</buildtool_depend>`을 `<buildtool_depend>ament_cmake</buildtool_depend>`으로 변경해야 합니다.
2.  **`CMakeLists.txt` 수정**: ROS2 `ament_cmake` 스타일에 맞게 수정해야 합니다.
3.  **`display.launch` 파일 수정**: `roslaunch` XML 형식을 Python 기반의 ROS2 launch 파일로 변경해야 합니다.

추후 `robot-link-forge`에서 ROS2용 내보내기 기능이 추가될 수 있습니다.
