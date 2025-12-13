prompt
VS Code의 Gemini Pro Extension을 활용하여 효율적으로 개발하실 수 있도록, 3단계 마일스톤으로 나누어 프롬프트를 작성했습니다.
이 프롬프트들은 복사(Copy) -> 붙여넣기(Paste) 하시면 바로 코드를 생성해 주도록 최적화되어 있습니다.
📂 사전 준비 (Project Structure)
먼저 프로젝트 폴더를 하나 만들고, 터미널에서 아래와 같이 폴더를 구분해 주세요.
 * frontend/ : React 앱
 * backend/ : FastAPI 서버
1단계: 기본 골격 및 3D 에디터 (Primitive Shape & State)
목표: React + Three.js 환경을 구축하고, 박스/원통으로 로봇의 관절 구조(Parent-Child)를 시각화하며, 웹상에서 조인트를 움직여보는 단계입니다.
📋 Prompt 1-1: Frontend 프로젝트 설정 및 3D 뷰어
(VS Code에서 frontend 폴더를 열고 입력하세요)
현재 폴더에 React, Vite, TypeScript, Tailwind CSS, React Three Fiber(R3F), Drei, Leva, Zustand를 사용하는 로봇 URDF 빌더 웹 앱의 기본 구조를 만들어줘.

요구사항:
1. `useRobotStore` (Zustand): 로봇의 링크와 조인트 데이터를 트리 구조로 관리.
   - 초기 데이터: Base Link(Box 형태) 하나만 존재.
   - 각 링크/조인트는 고유 ID, position, rotation, type, shape(box/cylinder), dimensions 속성을 가짐.
2. `RobotVisualizer` (R3F Component): Store의 데이터를 기반으로 재귀적(Recursive)으로 로봇을 렌더링.
   - Link는 <Box> 또는 <Cylinder> 메쉬로 렌더링.
   - Joint는 <Group>으로 감싸고, Drei의 <TransformControls>를 사용하거나 Leva 패널을 통해 로컬 회전(Rotation)을 제어할 수 있어야 함.
3. `App.tsx`: 화면을 좌측(트리 구조 및 추가 버튼), 중앙(3D Canvas), 우측(속성 편집기)으로 3분할 레이아웃 적용.

위 내용을 구현하는 핵심 코드들(App.tsx, store.ts, RobotVisualizer.tsx)을 작성해 줘.

📋 Prompt 1-2: 인터랙티브 UI (조인트/링크 추가 기능)
(위 코드가 생성된 후 이어서 입력)
이제 좌측 패널과 우측 패널의 기능을 구현해 줘.

1. 좌측 패널 (Hierarchy & Actions):
   - 현재 로봇의 계층 구조를 트리 리스트로 보여줘.
   - 각 항목 옆에 'Add Child Joint' 또는 'Add Child Link' 버튼을 추가해 줘.
   - 버튼 클릭 시 `useRobotStore`에 새로운 조인트/링크 객체가 추가되고 3D 화면에 즉시 박스/원통이 붙어서 나타나야 함.
2. 우측 패널 (Inspector):
   - 3D 뷰나 좌측 트리에서 특정 링크/조인트를 클릭하면 'Selected' 상태가 됨.
   - 선택된 항목의 크기(Dimension), 색상, 조인트 각도 등을 수정할 수 있는 입력 폼(Input)을 보여줘.
   - 입력 값이 바뀌면 3D 모델이 실시간으로 업데이트되도록 바인딩해 줘.

> ✅ 확인 포인트 (Checklist)
>  * npm run dev 실행 시 화면이 뜨는가?
>  * 'Add' 버튼을 누르면 회색 박스/원통이 계속 이어져서 생성되는가?
>  * 우측 패널에서 슬라이더를 움직이면 관절이 꺾이는가?
> 
2단계: STL 파일 업로드 및 서버 연동 (Backend & STL)
목표: Python 백엔드를 구축하여 기구팀이 만든 STL 파일을 업로드하고, 3D 뷰어의 박스를 실제 STL 모델로 교체합니다.
📋 Prompt 2-1: FastAPI 백엔드 구축 및 파일 업로드
(VS Code에서 backend 폴더를 열고 입력하세요)
Python FastAPI를 사용하여 백엔드 서버를 구축해 줘.

요구사항:
1. `main.py`: 기본 서버 파일. CORS 설정(프론트엔드 5173 포트 허용) 포함.
2. STL 파일 업로드 API (`POST /upload`):
   - 사용자가 STL 파일을 업로드하면 서버의 `static/meshes/` 폴더에 저장.
   - 저장된 파일에 접근할 수 있는 정적 파일 URL을 반환 (예: `http://localhost:8000/static/meshes/base_link.stl`).
3. 프로젝트 구조를 잡고, `uvicorn`으로 실행 가능한 코드를 작성해 줘.

📋 Prompt 2-2: 프론트엔드 STL 로더 연동
(다시 frontend 코드로 돌아와서 입력)
프론트엔드의 `Inspector` (우측 패널) 컴포넌트와 `RobotVisualizer`를 수정해서 STL 기능을 추가해 줘.

1. 우측 패널 수정:
   - 링크(Link) 선택 시 'Upload STL' 버튼 추가.
   - 파일을 선택하면 백엔드(`http://localhost:8000/upload`)로 전송.
   - 서버로부터 받은 URL을 해당 링크의 `meshUrl` 속성에 저장.
2. 3D 뷰어 수정:
   - 링크 데이터에 `meshUrl`이 있으면, 기존 Box/Cylinder 대신 R3F의 `useLoader`와 `STLLoader`를 사용하여 해당 STL 파일을 렌더링.
   - STL 로딩 중 에러를 방지하기 위해 `<Suspense>` 처리도 포함해 줘.

> ✅ 확인 포인트 (Checklist)
>  * backend에서 서버 실행(uvicorn main:app --reload), frontend 실행 상태인가?
>  * 웹 UI에서 STL 파일을 업로드하면 backend/static/meshes 폴더에 파일이 생기는가?
>  * 업로드 직후 웹 3D 화면의 박스가 멋진 로봇 부품으로 변하는가?
> 
3단계: ROS 패키지 구조화 및 Export (URDF/Xacro/MJCF)
목표: 현재 작업물을 SW팀이 바로 쓸 수 있는 형태(ZIP)로 내보냅니다.
📋 Prompt 3-1: URDF/Xacro 생성 및 ROS 패키지 패킹 (Backend)
(백엔드 코드 작업)
FastAPI 백엔드에 'Export ROS Package' 기능을 추가하려고 해. Jinja2 템플릿을 사용할 거야.

요구사항:
1. 데이터 수신: 프론트엔드에서 현재 로봇의 전체 JSON 트리 데이터(링크, 조인트, STL 파일명 등)를 `POST /export`로 받음.
2. 파일 생성 로직 (`export_manager.py` 등):
   - 임시 폴더에 ROS 패키지 표준 구조 생성 (`urdf/`, `meshes/`, `launch/`, `rviz/`, `CMakeLists.txt`, `package.xml`).
   - **URDF 생성:** JSON 트리를 순회하며 표준 URDF XML 작성.
   - **Xacro 생성:** 주요 수치(길이, 질량)를 `<xacro:property>`로 뺀 `.xacro` 파일도 같이 생성.
   - **Launch 파일:** `robot_state_publisher`와 `rviz`를 실행하는 `display.launch` 자동 생성.
   - **Meshes:** 사용자가 업로드했던 STL 파일들을 이 패키지의 `meshes/` 폴더로 복사.
3. 압축 및 반환: 생성된 전체 폴더를 ZIP으로 압축하여 클라이언트에게 다운로드 제공.

위 로직을 수행하는 파이썬 코드와 템플릿 예시를 작성해 줘.

📋 Prompt 3-2: MuJoCo(MJCF) 변환 기능 추가 (Advanced)
(백엔드 코드 작업 - 옵션)
방금 만든 Export 기능에 MuJoCo(MJCF) 포맷 지원도 추가해 줘.

1. URDF to MJCF 변환:
   - 사용자가 보낸 로봇 데이터 혹은 생성된 URDF를 기반으로 `mujoco` 호환 XML을 생성하는 로직 추가.
   - MuJoCo는 조인트 정의 방식이 조금 다르므로(body 안에 joint 포함), 이를 고려해 재귀적으로 XML 태그를 생성하는 함수를 만들어줘.
2. 결과물 포함:
   - ZIP 파일 내보낼 때 `mujoco/` 폴더를 만들고 그 안에 `robot.xml` (MJCF 파일)을 같이 포함시켜 줘.

> ✅ 확인 포인트 (Checklist)
>  * 프론트엔드에 'Download Package' 버튼을 만들고 백엔드 API를 호출했는가?
>  * 다운로드된 ZIP 파일을 풀었을 때 urdf, meshes, launch 폴더가 예쁘게 들어있는가?
>  * 터미널에서 check_urdf robot.urdf (ROS 설치 시) 명령어로 문법 오류가 없는지 확인.
>  * 생성된 display.launch를 실행했을 때 RViz가 뜨고 로봇이 보이는가?
> 
💡 Tip: VS Code Gemini Pro Extension 사용 팁
 * 한 번에 하나씩: 위 프롬프트를 한 번에 다 넣지 마시고, 1-1이 완료되고 코드가 정상 작동하면 1-2를 넣으세요.
 * 파일 분리: Gemini가 코드를 너무 길게 짜주면 "이 부분을 components/Sidebar.tsx로 분리해 줘"라고 요청하여 코드를 깔끔하게 유지하세요.
 * 에러 발생 시: 에러 로그를 그대로 복사해서 "이런 에러가 나는데 고쳐줘"라고 하면 아주 잘 고쳐줍니다.
