알겠습니다. **"너무 간단해서 짜다 만 코드"**가 나오지 않도록, 기능 명세서(Spec Sheet) 수준으로 아주 구체적인 Step 1 전용 프롬프트를 작성해 드리겠습니다.
Gemini Pro가 코드를 작성할 때 데이터 구조(Store), 3D 렌더링 로직(R3F), 그리고 UI 인터랙션(React) 이 유기적으로 연결되도록 강제하는 것이 핵심입니다.
아래 내용을 복사해서 VS Code 확장 프로그램에 입력해 주세요.
📋 Step 1. 상세 개발 프롬프트: 핵심 데이터 구조 및 실시간 3D 에디터
Role: 너는 숙련된 Three.js 및 React 프론트엔드 개발자야.
Goal: 로봇의 URDF 구조를 웹에서 설계하는 'RobotLinkForge'의 가장 기초적이지만 완벽하게 작동하는 프로토타입(Step 1) 을 만들어줘.
이번 단계에서는 서버 연결이나 파일 저장은 신경 쓰지 말고, 오직 클라이언트 사이드에서 작동하는 편집 기능과 3D 시각화에 집중해 줘.
1. 상태 관리 (Zustand Store: store.ts)
가장 먼저 데이터 구조를 잡아야 해. 단순히 배열이 아니라 트리 구조 탐색이 가능한 형태여야 해.
 * Types:
   * Link: { id: string, name: string, visual: { type: 'box' | 'cylinder' | 'sphere', dimensions: [x, y, z], color: string }, childJoints: string[] }
   * Joint: { id: string, name: string, parentLinkId: string, childLinkId: string, type: 'revolute' | 'fixed', origin: { xyz: [x,y,z], rpy: [r,p,y] }, axis: [x,y,z], limit: { lower, upper }, currentAngle: number }
 * Store Actions (State Logic):
   * addJoint(parentLinkId):
     * 부모 링크 ID를 받아서, 새로운 Joint와 그에 연결된 새로운 Link를 동시에 생성해.
     * 중요: 새로 생성된 Joint의 origin은 부모 링크의 크기를 고려해서 자동으로 겹치지 않는 위치(예: z축으로 0.5 이동)에 생성되도록 기본값을 잡아줘.
   * updateJoint(id, field, value): 특정 조인트의 오리진(xyz, rpy), 축(axis), 현재 각도(currentAngle)를 수정하는 함수.
   * updateLink(id, field, value): 링크의 크기나 시각적 타입을 수정하는 함수.
   * selectItem(id, type): 현재 사용자가 편집 중인 항목을 설정.
 * Helper:
   * 데이터가 업데이트될 때 콘솔에 로그를 찍어줘.
2. 재귀적 3D 뷰어 (R3F Component: RobotVisualizer.tsx)
로봇의 Kinematics(운동학)를 시각적으로 구현해야 해.
 * 컴포넌트 구조 (RecursiveLink):
   * 이 컴포넌트는 linkId를 props로 받아서 렌더링해.
   * 렌더링 순서:
     * 현재 Link의 Mesh(Box/Cylinder)를 그린다.
     * onClick 이벤트: e.stopPropagation()을 사용하여 클릭 시 해당 링크가 '선택'되도록 Store의 selectItem을 호출한다. 선택된 객체는 색상을 밝게(Hot Pink 등) 변경하여 표시한다.
     * childJoints 배열을 순회하며 하위 JointWrapper 컴포넌트를 렌더링한다.
 * 조인트 래퍼 (JointWrapper):
   * 이 컴포넌트는 jointId를 props로 받는다.
   * 좌표 변환 (중요):
     * Group을 만들고 position={origin.xyz}와 rotation={origin.rpy}를 적용하여 부모 링크로부터의 오프셋(Orientation) 을 잡는다.
     * 그 내부에서 또 다른 Group을 만들고, 이번에는 joint.axis를 기준으로 currentAngle만큼 회전시킨다 (실제 움직임).
   * Visual Helpers:
     * 조인트 위치에 AxesHelper(RGB 화살표)를 무조건 표시해서 사용자가 좌표축 방향을 알 수 있게 해줘.
     * onClick 시 조인트 자체도 선택 가능해야 해 (작은 Sphere로 표시).
   * 마지막으로 이 내부에서 childLinkId를 가진 RecursiveLink를 다시 호출한다 (재귀).
3. 통합 GUI (App.tsx & Sidebar)
화면 구성을 잡고 실시간 제어 기능을 넣어줘.
 * Layout: 전체 화면을 Canvas(전체)와 Sidebar(우측 상단 고정 패널, 폭 320px)로 구성.
 * Sidebar Content (Inspector):
   * 현재 selectedItem이 없으면 "Select a Link or Joint" 메시지 표시.
   * Link 선택 시:
     * 이름 변경 Input.
     * Visual Type 변경 (Box/Cylinder).
     * Dimensions (X, Y, Z) 입력창 -> 입력 즉시 3D 메쉬 크기 변경.
     * Action: [+ Add Child Joint] 버튼 -> 누르면 현재 링크 끝에 새 관절 생성.
   * Joint 선택 시:
     * Origin Control: X, Y, Z (위치) / Roll, Pitch, Yaw (회전) 입력창 (Step 0.1 단위). 값이 바뀌면 3D 상에서 자식 링크 덩어리가 통째로 이동해야 함.
     * Axis Control: X, Y, Z 벡터 입력 (예: 1, 0, 0 이면 X축 회전).
     * Test Driver: Current Angle 슬라이더 (Range: -3.14 ~ 3.14). 이 슬라이더를 움직이면 로봇 팔이 꺾여야 함.
[요청 사항]
위 설계를 바탕으로 store.ts, RobotVisualizer.tsx, Sidebar.tsx, App.tsx 의 전체 코드를 작성해 줘.
CSS는 Tailwind class를 사용하고, 아이콘은 lucide-react를 사용해 줘.
코드는 복잡하더라도 파일별로 분리해서 작성해 줘.
💡 이 프롬프트가 챙기는 디테일 (Point Check)
 * "오리엔테이션, 위치, 회전각": Sidebar 섹션의 Origin Control과 Axis Control에서 구체적으로 입력 필드를 요구했습니다.
 * "가이드 (Helpers)": 3D 뷰어 섹션에서 **AxesHelper**를 필수로 넣어달라고 했습니다. 기구 개발자는 축이 어디를 향하는지 눈으로 봐야 설계할 수 있기 때문입니다.
 * "중앙 GUI 선택 및 수정": 3D 뷰어 섹션에서 **onClick 이벤트와 e.stopPropagation()**을 명시했습니다. 이게 없으면 손가락을 클릭했는데 팔뚝이 선택되는 버그가 생깁니다.
 * "실시간 업데이트": onChange 이벤트가 Store를 건드리고, R3F가 이를 구독(Subscribe)하는 구조로 코드를 짜게 됩니다.
 * "움직여 보는 기능": Test Driver 슬라이더를 명시하여, 애니메이션 없이도 수동으로 관절을 꺾어볼 수 있게 했습니다.
이 프롬프트를 입력하면, 단순히 네모 박스만 나오는 게 아니라, 클릭하면 속성창이 뜨고 슬라이더를 밀면 팔이 꺾이는 수준의 코드가 나올 것입니다.
