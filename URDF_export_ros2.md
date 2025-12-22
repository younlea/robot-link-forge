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

## 오류 해결 가이드 (Troubleshooting)

ROS2 패키지를 빌드하거나 실행할 때 발생할 수 있는 몇 가지 일반적인 문제와 해결 방법입니다.

### 1. `ros2 launch` 실행 시 "package not found" 오류

-   **증상**: `ros2 launch <package_name> display.launch.py` 실행 시 `package '<package_name>' not found` 와 같은 오류가 발생합니다.
-   **원인**: 현재 터미널 세션에 ROS2 환경이 제대로 설정되지 않았습니다.
-   **해결 방법**:
    1.  ROS2 워크스페이스의 `setup.bash` 파일을 소싱(source)합니다.
        ```bash
        source ~/ros2_ws/install/setup.bash
        ```
    2.  새 터미널을 열 때마다 이 작업을 반복하지 않으려면 `~/.bashrc` 파일에 위 명령어를 추가하세요.

### 2. 의존성 패키지 관련 오류 (`joint_state_publisher_gui` 등)

`colcon build` 또는 `ros2 launch` 실행 중 `joint_state_publisher_gui`, `rviz2` 등의 패키지를 찾을 수 없다는 오류가 발생할 수 있습니다.

#### 단계 1: 패키지 설치
먼저 `apt`를 사용하여 필요한 패키지를 설치합니다.
```bash
sudo apt-get install ros-humble-joint-state-publisher-gui
```

#### 단계 2: `apt` 설치 시 "404 Not Found" 오류
-   **증상**: `apt install` 실행 시 패키지를 찾을 수 없다는(404) 오류가 발생합니다.
-   **원인**: 로컬 패키지 목록이 최신이 아닙니다.
-   **해결 방법**: 패키지 목록을 업데이트한 후 다시 설치를 시도합니다.
    ```bash
    sudo apt-get update
    sudo apt-get install ros-humble-joint-state-publisher-gui
    ```

#### 단계 3: `apt update` 실행 시 "EXPKEYSIG" (서명 오류)
-   **증상**: `apt update` 실행 시 `EXPKEYSIG` 또는 `다음 서명이 올바르지 않습니다` 와 같은 GPG 서명 오류가 발생합니다.
-   **원인**: ROS 저장소의 보안 키가 만료되었습니다.
-   **해결 방법**: 다음 명령어를 실행하여 새 보안 키로 업데이트합니다.
    ```bash
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    ```
    키 업데이트 후, 다시 `sudo apt-get update` 와 `sudo apt-get install ...`을 실행하세요.

#### 단계 4: 패키지 설치 후에도 `ros2 launch`에서 찾지 못하는 경우
-   **증상**: `apt`로 패키지를 성공적으로 설치했지만, `ros2 launch` 실행 시 여전히 패키지를 찾지 못한다고 나옵니다.
-   **원인**: 터미널 환경이 시스템 전체에 설치된 ROS 패키지(`_`/opt/ros/humble`_)를 인식하지 못하고, 로컬 워크스페이스만 바라보고 있기 때문입니다.
-   **해결 방법**: `/opt/ros/humble/setup.bash` 파일을 소싱하여 시스템 ROS 환경을 터미널에 적용합니다.
    ```bash
    source /opt/ros/humble/setup.bash
    ```
    이후 다시 `ros2 launch`를 실행합니다. 근본적인 해결을 위해서는 이 명령어를 `~/.bashrc` 파일의 **로컬 워크스페이스 `source` 명령어보다 먼저** 추가하는 것을 권장합니다.