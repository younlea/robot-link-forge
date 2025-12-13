네, 아주 좋은 접근입니다. VS Code Gemini Pro Extension에게 **"우리가 만들려는 전체 그림(Master Plan)"**을 한 번에 명확히 주입하고, 단계별로 코드를 짜게 하는 것이 가장 효율적입니다.
앞서 논의한 다중 분기(Branching), 가상 링크(Virtual Link), STL 로드, ROS 패키지 Export 기능을 모두 통합하여 하나의 마스터 프롬프트로 정리했습니다.
이 내용을 복사해서 Gemini에게 전달하시면 됩니다.

```
📋 RobotLinkForge 개발을 위한 마스터 프롬프트
아래 내용을 VS Code 채팅창에 그대로 붙여넣으세요.
Role: 너는 React, Three.js(R3F), Python(FastAPI)에 능숙한 풀스택 로보틱스 툴 개발자야.
Goal: 기구 개발자와 SW 개발자가 협업할 수 있는 웹 기반 로봇 모델링 도구 **'RobotLinkForge'**를 구축하려고 해.
이 툴은 사용자가 웹에서 로봇의 관절(Joint)과 링크(Link)를 조립하고, STL 파일을 입히고, 최종적으로 시뮬레이션 가능한 ROS 패키지(URDF/MJCF)로 내보내는 기능을 제공해야 해.
우리는 아래의 4단계 마일스톤으로 개발을 진행할 거야. 각 단계의 요구사항을 숙지하고, 내가 요청할 때마다 해당 단계의 코드를 작성해 줘.
🛠️ 기술 스택 (Tech Stack)
 * Frontend: React, Vite, TypeScript, Tailwind CSS, Zustand (State Management)
 * 3D Graphics: React Three Fiber (R3F), Drei, Leva
 * Backend: Python FastAPI
 * Data Format: URDF 표준을 따르는 JSON 트리 구조
📅 단계별 상세 요구사항 (Milestones)
Step 1: 핵심 데이터 구조 및 3D 뷰어 (Core & Visualizer)
가장 중요한 단계야. 로봇 손(Hand)처럼 한 링크에서 여러 손가락이 뻗어 나가는 '다중 분기(Multi-Branching)' 구조를 지원해야 해.
 * Zustand Store (useRobotStore):
   * Link Interface:
     * id (UUID), name, visual ({ type: 'box' | 'cylinder' | 'sphere' | 'none', dimensions, color, meshUrl })
     * childJoints: string[] (이 링크에 연결된 자식 조인트들의 ID 리스트 -> 다중 분기 지원 핵심)
   * Joint Interface:
     * id, name, type (revolute, fixed, prismatic 등)
     * parentLinkId, childLinkId
     * origin: { xyz: [x,y,z], rpy: [r,p,y] } (부모 링크로부터의 상대적 위치/회전 오프셋)
     * axis, limits, currentValue (현재 각도)
   * 기능: visual type이 'none'일 경우 메쉬는 그리지 않지만 좌표계 연결은 유지해야 함 (가상 링크 지원).
 * 3D Visualizer (RobotVisualizer Component):
   * BaseLink부터 시작하여 재귀적(Recursive)으로 렌더링.
   * 렌더링 로직: Link 렌더링 -> childJoints 순회 -> 각 Joint의 origin만큼 이동/회전한 <Group> 생성 -> Joint 축 표시 -> 연결된 Child Link 렌더링 (재귀).
Step 2: 인터랙티브 에디터 UI (GUI)
 * Layout: 좌측(트리), 중앙(3D 뷰), 우측(인스펙터).
 * Hierarchy Tree (Left):
   * 트리 구조 시각화.
   * Link 선택 시: [+] Add Joint 버튼 노출. (클릭 시 새로운 Joint와 Child Link 생성).
   * 삭제 기능: 하위 트리 전체 삭제 (Cascade Delete).
 * Inspector (Right):
   * Joint 선택 시: Origin (XYZ, RPY) 정밀 수정 입력창, Joint Limit, 테스트용 슬라이더 제공.
   * Link 선택 시: Visual Type 변경 (Box <-> None), 색상 변경.
   * 실시간 반영: 입력 즉시 3D 뷰어 업데이트.
Step 3: STL 파일 업로드 및 백엔드 연동
 * Backend (FastAPI):
   * POST /upload: STL 파일을 받아 static/meshes/에 저장하고 접근 URL 반환.
 * Frontend:
   * Inspector에 'Upload STL' 버튼 추가.
   * 업로드 완료 시 해당 Link의 visual.type을 'mesh'로 변경하고 meshUrl 업데이트.
   * R3F에서 useLoader(STLLoader, url)를 통해 렌더링.
Step 4: ROS 패키지 Export (Final)
 * Backend Logic:
   * POST /export: 전체 로봇 JSON 데이터를 수신.
   * ROS 표준 패키지 구조 생성: urdf/, meshes/, launch/, rviz/, CMakeLists.txt, package.xml.
   * URDF Generation: JSON 트리를 순회하며 XML 생성 (STL 경로를 package://...로 자동 변환).
   * RViz & Launch: display.launch (Joint State Publisher GUI + Robot State Publisher + RViz) 자동 생성.
 * Output: 생성된 폴더를 .zip으로 압축하여 다운로드 제공.
작업 지시:
위의 Step 1 (핵심 데이터 구조 및 3D 뷰어) 부터 코드를 작성해 줘.
App.tsx, store.ts, RobotVisualizer.tsx 파일을 중심으로 작성하고, 다중 분기와 가상 링크(Visual None)가 잘 작동하도록 신경 써줘.
💡 사용 가이드
 * 위 내용을 복사해서 Gemini에게 붙여넣으세요.
 * Gemini가 Step 1 코드를 작성해주면, 프로젝트에 적용하고 실행해보세요.
   * 확인 포인트: 화면에 박스가 뜨는지, Add Joint를 여러 번 눌렀을 때 가지치기가 되는지 확인.
 * Step 1이 잘 돌아가면, 채팅창에 "이제 Step 2 UI 기능을 구현해 줘" 라고 입력하세요. (이미 마스터 프롬프트에 내용이 있어서 찰떡같이 알아듣습니다.)
 * 이어서 Step 3, Step 4도 순차적으로 요청하시면 됩니다.


```




