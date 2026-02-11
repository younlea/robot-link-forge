# 🏗️ Tendon / Rolling Contact / Obstacle 확장 구현 계획

> **작성일**: 2026-02-11  
> **목적**: 기존 Direct Joint 시스템에 텐던 구동, 롤링 컨택 조인트, 장애물 상호작용 기능을 추가  
> **원칙**: 기존 Motion Recording, Replay, Torque Graph 기능을 절대 깨뜨리지 않음

---

## 📊 현재 코드베이스 분석 요약

### 아키텍처 흐름
```
[Frontend: React + Three.js]  →  [Backend: FastAPI]  →  [Export: URDF/MJCF/Gazebo]
     │                                    │                        │
     ├── types.ts (데이터 모델)            ├── robot_models.py      ├── mjcf_exporter.py (925줄)
     ├── store.ts (Zustand, 1817줄)       ├── main.py (2580줄)     ├── urdf_exporter.py
     ├── Sidebar.tsx (1480줄)              │                        ├── motion_exporter.py (4377줄)
     ├── RobotVisualizer.tsx (817줄)       │                        ├── gazebo_exporter.py
     └── RecordingPanel.tsx (786줄)        └── motor_validation/    └── stl_utils.py
                                                ├── motor_physics.py (오케스트레이터)
                                                ├── pid_controller.py
                                                ├── friction_model.py
                                                └── efficiency_model.py
```

### 현재 Joint 처리 구조

| 계층 | 파일 | 현재 지원 타입 |
|------|------|---------------|
| **Frontend Type** | `types.ts` L27 | `'fixed' \| 'rotational' \| 'prismatic'` |
| **UI Selector** | `Sidebar.tsx` ~L507 | Fixed, Rotational, Prismatic (3개 옵션) |
| **Backend Model** | `robot_models.py` L39 | `type: str` (자유 문자열) |
| **MJCF Export** | `mjcf_exporter.py` | `rotational`→hinge, `prismatic`→slide, `fixed`→no joint |
| **URDF Export** | `urdf_exporter.py` | rotational→revolute (multi-DOF split), prismatic→prismatic |
| **Motor Physics** | `motor_physics.py` | Direct coupling only (PID → T-N → Efficiency → Friction) |

### 핵심 발견
- `JointType`에 `'tendon' | 'rollingContact'`가 이미 추가되어 있었으나 **UI에는 미노출**
- 현재 `store.ts`의 `createDefaultJoint`에 `tendon`/`rollingContact` 분기가 있었으나 **불완전하고 원복됨**
- Tendon은 Joint이 아닌 **별도 시스템**으로 모델링해야 함 (MuJoCo의 `<tendon>` 섹션)
- Obstacle은 현재 전혀 없는 **신규 엔티티**

---

## 🎯 설계 방향 결정

### ❌ 잘못된 접근: "Tendon을 JointType에 넣기"
이전 시도에서 `JointType = '... | 'tendon'`으로 추가했으나 이는 **개념적으로 틀림**:
- MuJoCo에서 Tendon은 `<joint>`가 아닌 `<tendon><spatial>` 섹션
- Tendon은 **여러 조인트를 연결**하는 전송 메커니즘
- Joint ≠ Tendon, Tendon은 Joint 위에 존재하는 별도 레이어

### ✅ 올바른 접근: 3개 독립 시스템
```
기존: Link → Joint (Direct Motor) → Link
확장:
  1. Link → Joint (Rolling Contact) → Link   ← JointType 확장
  2. Tendon System (별도 엔티티)               ← 새로운 데이터 모델
  3. Obstacle System (별도 엔티티)             ← 새로운 데이터 모델
```

---

## 📋 Phase별 상세 구현 계획

### Phase 1: 데이터 모델 리팩토링

#### 1-A. Frontend `types.ts` 변경

```typescript
// ── JointType 수정 (tendon 제거, rolling 추가) ──
export type JointType = 'fixed' | 'rotational' | 'prismatic' | 'rolling';

// ── Rolling Joint 전용 속성 ──
export interface RollingContactParams {
  curvatureRadius: number;    // 곡률 반경 (m)
  contactFriction: number;    // 접촉 마찰 계수
  surfaceType: 'convex' | 'concave';
}

// ── RobotJoint에 추가 ──
export interface RobotJoint {
  // ...기존 필드 유지...
  rollingParams?: RollingContactParams;  // rolling 타입일 때만 사용
}

// ── 텐던 시스템 (완전 새로운 엔티티) ──
export interface TendonRoutingPoint {
  linkId: string;
  localPosition: [number, number, number];  // 해당 Link 좌표계 기준
  siteId: string;                            // MJCF site name
}

export interface Tendon {
  id: string;
  name: string;
  type: 'active' | 'passive';

  // 경로 정의
  routingPoints: TendonRoutingPoint[];

  // 물리 파라미터
  stiffness: number;     // N/m (Passive 텐던 필수)
  damping: number;       // N·s/m
  restLength: number;    // 자연 길이 (m)

  // Active 텐던 전용
  actuatorMotorId?: string;  // 어떤 모터가 이 텐던을 구동하는지
  momentArm?: number;        // 유효 모멘트 암 (m)

  // 시각화
  color: string;
  width: number;           // 선 두께 (시각화 전용)
}

// ── 장애물 시스템 ──
export interface Obstacle {
  id: string;
  name: string;
  shape: 'box' | 'sphere' | 'cylinder';
  dimensions: [number, number, number];
  position: [number, number, number];
  rotation: [number, number, number];  // RPY
  color: string;
  physics: {
    friction: number;
    solref: [number, number];    // MuJoCo contact solver ref
    solimp: [number, number, number];
  };
  enabled: boolean;  // 토글로 on/off
}

// ── 센서 시스템 ──
export interface Sensor {
  id: string;
  type: 'touch' | 'force';
  linkId: string;
  localPosition: [number, number, number];
  localRotation: [number, number, number];
  siteName: string;  // MJCF export용
}

// ── RobotState 확장 ──
export interface RobotState {
  // ...기존 필드 전부 유지...
  tendons: Record<string, Tendon>;        // 새로 추가
  obstacles: Record<string, Obstacle>;    // 새로 추가
  sensors: Record<string, Sensor>;        // 새로 추가
}
```

#### 1-B. Backend `robot_models.py` 변경

```python
# 새로운 Pydantic 모델 추가
class RollingContactParams(BaseModel):
    curvatureRadius: float
    contactFriction: float
    surfaceType: str = "convex"

class TendonRoutingPoint(BaseModel):
    linkId: str
    localPosition: List[float]
    siteId: str

class Tendon(BaseModel):
    id: str
    name: str
    type: str  # 'active' | 'passive'
    routingPoints: List[TendonRoutingPoint]
    stiffness: float = 0.0
    damping: float = 0.0
    restLength: float = 0.0
    actuatorMotorId: Optional[str] = None
    momentArm: Optional[float] = None
    color: str = "#ff6600"
    width: float = 0.002

class ObstaclePhysics(BaseModel):
    friction: float = 0.5
    solref: List[float] = [0.02, 1.0]
    solimp: List[float] = [0.9, 0.95, 0.001]

class Obstacle(BaseModel):
    id: str
    name: str
    shape: str
    dimensions: List[float]
    position: List[float]
    rotation: List[float]
    color: str = "#888888"
    physics: ObstaclePhysics = ObstaclePhysics()
    enabled: bool = True

class SensorDef(BaseModel):
    id: str
    type: str  # 'touch' | 'force'
    linkId: str
    localPosition: List[float]
    localRotation: List[float] = [0, 0, 0]
    siteName: str

# RobotData 확장 (하위 호환 유지)
class RobotData(BaseModel):
    links: Dict[str, RobotLink]
    joints: Dict[str, RobotJoint]
    baseLinkId: str = Field(alias='baseLinkId')
    tendons: Optional[Dict[str, Tendon]] = {}        # 새로 추가, Optional로 하위호환
    obstacles: Optional[Dict[str, Obstacle]] = {}    # 새로 추가
    sensors: Optional[Dict[str, SensorDef]] = {}     # 새로 추가

# RobotJoint 확장
class RobotJoint(BaseModel):
    # ...기존 필드 유지...
    rollingParams: Optional[RollingContactParams] = None  # 추가
```

#### 1-C. 수정 영향도 분석

| 파일 | 변경 유형 | 위험도 | 이유 |
|------|----------|--------|------|
| `types.ts` | 타입 확장 | 🟢 Low | 새 필드는 Optional, 기존 코드 영향 없음 |
| `robot_models.py` | 모델 확장 | 🟢 Low | 새 필드는 Optional + default 값 |
| `store.ts` | State + Actions 추가 | 🟡 Med | `createInitialState`에 새 빈 맵 추가 필요 |
| `Sidebar.tsx` | UI 패널 추가 | 🟡 Med | 기존 Joint Inspector는 건드리지 않고 새 섹션 추가 |
| `mjcf_exporter.py` | Export 확장 | 🔴 High | 핵심 변경, 조건분기 추가 필요 |
| `motion_exporter.py` | Tendon 토크 매핑 | 🔴 High | 텐던 tension → torque 변환 로직 |
| `motor_physics.py` | 파이프라인 확장 | 🔴 High | Tendon moment arm 계산 추가 |

---

### Phase 2: GUI & Interaction (Three.js Viewer)

#### 2-A. Sidebar.tsx 변경 계획

**기존 Joint Type Selector 수정** (~L507):
```
현재:  Fixed | Rotational | Prismatic
변경:  Fixed | Rotational | Prismatic | Rolling Contact
```
- `'rolling'` 선택 시 추가 UI 표시: curvatureRadius, contactFriction, surfaceType
- `'tendon'`은 여기서 제거 (텐던은 별도 패널)

**새로운 사이드바 섹션 추가** (기존 섹션 아래):
```
┌──────────────────────────┐
│ 🔧 Joint Inspector       │  ← 기존 유지
├──────────────────────────┤
│ 🧵 Tendon System          │  ← 새로 추가
│  [+ Add Tendon]           │
│  ├ Tendon_1 (Active)     │
│  │  Motor: motor_01      │
│  │  Routing: 4 points    │
│  │  Stiffness: --        │
│  └ Tendon_2 (Passive)    │
│    Stiffness: 50 N/m     │
│    Rest Length: 0.1m      │
├──────────────────────────┤
│ 🪨 Obstacles              │  ← 새로 추가
│  [+ Box] [+ Sphere] [+Cyl]│
│  ├ Obstacle_1 (Box)      │
│  │  Friction: 0.5        │
│  │  ☑ Enabled            │
│  └ Obstacle_2 (Sphere)   │
├──────────────────────────┤
│ 📡 Sensors                │  ← 새로 추가
│  [Click link to place]    │
│  ├ Touch_1 on finger_tip │
│  └ Force_1 on palm       │
└──────────────────────────┘
```

#### 2-B. RobotVisualizer.tsx 변경 계획

| 기능 | 구현 방식 |
|------|----------|
| **텐던 경로 시각화** | `<Line>` (drei) 컴포넌트로 routingPoints를 연결하는 선 렌더링 |
| **텐던 라우팅 모드** | 클릭한 Link 표면 좌표를 routingPoint로 추가 (raycasting) |
| **Rolling Joint 시각화** | 기존 Joint과 동일하지만 아이콘/색상 구분 (곡면 표시) |
| **장애물 렌더링** | `<Box>`, `<Sphere>`, `<Cylinder>` + TransformControls 기즈모 |
| **센서 마커** | Link 표면에 작은 점/원 렌더링 |

#### 2-C. 새로운 컴포넌트 파일

```
src/frontend/src/components/
  ├── TendonEditor.tsx      ← 텐던 패널 + 라우팅 모드
  ├── ObstacleManager.tsx   ← 장애물 추가/편집 패널
  └── SensorPlacement.tsx   ← 센서 배치 패널
```

---

### Phase 3: Advanced MJCF Export

#### 3-A. `mjcf_exporter.py` 함수 시그니처 변경

```python
def generate_mjcf_xml(
    robot: RobotData,
    robot_name: str,
    mesh_files_map: Dict[str, str],
    unique_link_names: Dict[str, str],
    use_mesh_collision: bool = False,
    direct_hand: bool = False,
    mesh_dir: Optional[str] = None,
    # ── 새 파라미터 ──
    include_obstacles: bool = False,     # 장애물 포함 여부
    scenario: str = 'static',           # 'static' | 'dynamic'
) -> Tuple[str, List[Dict]]:
```

#### 3-B. MJCF 출력 구조 (확장 후)

```xml
<mujoco model="my_robot">
  <compiler angle="radian" meshdir="meshes" balanceinertia="true"/>
  <option timestep="0.001" .../>

  <!-- 기존 contact 섹션 확장 -->
  <contact>
    <!-- 기존: parent-child exclusion pairs -->
    <exclude body1="..." body2="..."/>
    <!-- 새로: 장애물-로봇 contact pairs (include_obstacles=true일 때만) -->
    <pair geom1="robot_finger_tip" geom2="obstacle_1_geom"
          friction="0.8" solref="0.02 1" solimp="0.9 0.95 0.001"/>
  </contact>

  <asset>
    <!-- 기존 mesh assets -->
  </asset>

  <worldbody>
    <!-- 기존 로봇 body tree -->
    <body name="fixed_world" pos="0 0 0.5">
      <!-- ... -->
      <body name="finger_link">
        <!-- Rolling Joint: hinge로 근사하되 curvature 주석 -->
        <joint name="rolling_joint_1" type="hinge" axis="1 0 0"
               range="-1.57 1.57" damping="0.5"/>
        <!-- curvatureRadius=0.02 surfaceType=convex -->

        <!-- 센서용 site (Link.sensors에서 생성) -->
        <site name="touch_site_1" pos="0.01 0 0" size="0.005"/>
      </body>
    </body>

    <!-- 장애물 (include_obstacles=true일 때만) -->
    <body name="obstacle_1" pos="0.1 0 0.3" euler="0 0 0" mocap="true">
      <geom name="obstacle_1_geom" type="box" size="0.05 0.05 0.05"
            rgba="0.8 0.2 0.2 0.8" friction="0.5"
            solref="0.02 1" solimp="0.9 0.95 0.001"/>
    </body>
  </worldbody>

  <!-- ★ 새로: 텐던 섹션 -->
  <tendon>
    <!-- Passive 텐던: 스프링으로만 작용, 액추에이터 없음 -->
    <spatial name="tendon_passive_1" stiffness="50" damping="0.1"
             springlength="0.08">
      <site site="routing_site_1"/>
      <site site="routing_site_2"/>
      <site site="routing_site_3"/>
    </spatial>

    <!-- Active 텐던: 아래 actuator에서 구동 -->
    <spatial name="tendon_active_1">
      <site site="routing_site_4"/>
      <site site="routing_site_5"/>
    </spatial>
  </tendon>

  <actuator>
    <!-- 기존: Direct joint actuators -->
    <position name="act_joint1" joint="joint1" kp="50" kv="5"/>

    <!-- 새로: Active tendon actuator -->
    <general name="act_tendon_1" tendon="tendon_active_1"
             ctrlrange="-10 100" gainprm="1"/>
  </actuator>

  <sensor>
    <!-- 기존: touch sensors (leaf bodies) -->
    <touch name="touch_1" site="touch_site_1"/>

    <!-- 새로: 사용자 정의 센서 -->
    <touch name="custom_touch_1" site="custom_sensor_site_1"/>
    <force name="force_1" site="force_sensor_site_1"/>
  </sensor>
</mujoco>
```

#### 3-C. Rolling Joint Export 전략

MuJoCo에서 Rolling Contact Joint를 직접 지원하지 않으므로:

| 시뮬레이터 | 전략 |
|-----------|------|
| **Web Viewer** | Hinge로 근사 (기존 rotational과 동일하게 동작) |
| **MJCF (MuJoCo)** | `<joint type="hinge">`로 export + curvature 메타데이터를 XML 주석으로 삽입 |
| **Isaac Sim** | USD의 ArticulationJoint + custom property로 curvature 정보 전달 |

향후 고도화 시 MuJoCo의 `<equality><connect>` 제약조건을 활용하여 실제 rolling 물리를 구현할 수 있음.

---

### Phase 4: Analysis Tools 통합

#### 4-A. Torque Estimation 확장

**현재 파이프라인:**
```
PID(q_ref, q_act) → T_cmd → T-N Limit → Efficiency → Friction → T_final
```

**텐던 구동 시 추가 파이프라인:**
```
T_tendon_required = τ_joint / momentArm           ← 필요 텐던 장력
T_motor = T_tendon_required / gear_efficiency      ← 모터 출력 토크
모터 파이프라인: T_motor → T-N Limit → Efficiency → Friction → T_final
```

**수정 파일**: `motor_physics.py`
```python
class MotorPhysicsEngine:
    def __init__(self, motor, friction, pid_gains=None, tendon_config=None):
        self.tendon_config = tendon_config  # { moment_arm, friction_loss }

    def step(self, ...):
        if self.tendon_config:
            # Tendon-aware torque estimation
            required_tension = torque_cmd / self.tendon_config['moment_arm']
            motor_torque = required_tension / (1 - self.tendon_config['friction_loss'])
            # ... 기존 파이프라인에 motor_torque 주입
        else:
            # 기존 Direct 구동 파이프라인 (변경 없음)
            ...
```

#### 4-B. Motion Exporter 확장

**`motion_exporter.py`에 추가:**
- `generate_mujoco_tendon_replay_script()`: 텐던 기반 재생 스크립트
- 기존 `generate_mujoco_playback_script()`는 **그대로 유지** (Direct 조인트용)

---

### Phase 5: Backend API 확장

#### 새 endpoint

| Method | Endpoint | 용도 |
|--------|----------|------|
| 기존 유지 | `POST /api/export-mujoco-mjcf` | 파라미터 확장 (`include_obstacles`, `scenario`) |
| 신규 | `POST /api/export-isaac-sim` | Isaac Sim USD 포맷 export (향후) |

**`main.py` export-mujoco-mjcf endpoint 변경:**
```python
@app.post("/api/export-mujoco-mjcf")
async def export_mujoco_mjcf(...):
    # 기존 파라미터 유지
    # 추가 파라미터:
    include_obstacles = json.loads(form_data.get('include_obstacles', 'false'))
    scenario = form_data.get('scenario', 'static')
    # robot_data에서 tendons, obstacles, sensors 추출하여 exporter에 전달
```

---

## 🔀 구현 순서 & 의존성 그래프

```
Phase 1 (데이터 모델)
  ├── 1-A: types.ts        ─┐
  ├── 1-B: robot_models.py  ├── Phase 2에 필요
  └── 1-C: store.ts 확장   ─┘
                              │
Phase 2 (GUI)                 │
  ├── 2-A: Sidebar.tsx       ←┘
  ├── 2-B: RobotVisualizer.tsx
  ├── 2-C: TendonEditor.tsx (신규)
  ├── 2-D: ObstacleManager.tsx (신규)
  └── 2-E: SensorPlacement.tsx (신규)
                              │
Phase 3 (Export)              │
  ├── 3-A: mjcf_exporter.py ←┘ (Phase 1, 2 데이터 필요)
  ├── 3-B: motion_exporter.py
  └── 3-C: main.py API
                              │
Phase 4 (Analysis)            │
  ├── 4-A: motor_physics.py ←┘ (Phase 3 export 필요)
  └── 4-B: motor_validation 확장
```

---

## ⚠️ 위험 요소 & 완화 전략

| 위험 | 영향도 | 완화 전략 |
|------|--------|----------|
| store.ts 변경 시 기존 save/load 깨짐 | 🔴 High | 새 필드를 Optional + default 빈 객체로 추가. 기존 JSON 로드 시 없으면 {} |
| MJCF export 변경 시 기존 export 깨짐 | 🔴 High | 새 파라미터 모두 default=false. 기존 호출은 변경 없이 동작 |
| Motion Recording에 Tendon 데이터 미포함 | 🟡 Med | Recording에 tendon tension 값도 캡처하도록 확장 |
| 기존 프로젝트 파일 (.zip) 호환성 | 🟡 Med | loadRobot에서 tendons/obstacles/sensors 키 없으면 빈 객체로 초기화 |

---

## 📁 변경 대상 파일 총 정리

### 기존 파일 수정
| 파일 | 변경 내용 |
|------|----------|
| `src/frontend/src/types.ts` | Tendon, Obstacle, Sensor 인터페이스 추가, RobotState 확장 |
| `src/frontend/src/store.ts` | 초기 상태, CRUD 액션 추가, export 액션 파라미터 확장 |
| `src/frontend/src/components/Sidebar.tsx` | Rolling Contact UI, 텐던/장애물/센서 패널 |
| `src/frontend/src/components/RobotVisualizer.tsx` | 텐던 선, 장애물 렌더링, 센서 마커 |
| `src/backend/robot_models.py` | Tendon, Obstacle, SensorDef 모델 |
| `src/backend/main.py` | API 파라미터 확장, 새 데이터 전달 |
| `src/backend/exporters/mjcf_exporter.py` | tendon/obstacle/sensor/rolling XML 생성 |
| `src/backend/exporters/motion_exporter.py` | 텐던 재생 스크립트 |
| `src/backend/motor_validation/motor_physics.py` | Tendon-aware 토크 파이프라인 |

### 신규 파일 생성
| 파일 | 목적 |
|------|------|
| `src/frontend/src/components/TendonEditor.tsx` | 텐던 편집 UI |
| `src/frontend/src/components/ObstacleManager.tsx` | 장애물 관리 UI |
| `src/frontend/src/components/SensorPlacement.tsx` | 센서 배치 UI |

---

## 🎯 마일스톤

| 마일스톤 | 목표 | 검증 방법 |
|---------|------|----------|
| **M1** | 데이터 모델 완성, 기존 기능 정상 동작 | 기존 프로젝트 save/load/export 테스트 |
| **M2** | Rolling Contact UI + MJCF export | Rolling joint 설정 후 MJCF에 hinge로 정상 출력 |
| **M3** | 텐던 라우팅 + MJCF export | 텐던 경로 설정 → `<tendon><spatial>` 정상 출력 |
| **M4** | 장애물 + 조건부 export | Obstacle On/Off 토글 → MJCF 결과 검증 |
| **M5** | 센서 배치 + export | 센서 → `<sensor><touch>` 정상 출력 |
| **M6** | Torque estimation + Motor selection | 텐던 구동 시 Required Tension 표시 |
| **M7** | Isaac Sim export (향후) | USD 포맷 출력 검증 |

---

## 💡 논의 필요 사항

1. **텐던 라우팅 UI**: 3D 뷰에서 클릭으로 경로점을 추가하는 것이 좋은지, 아니면 사이드바에서 Link를 선택하여 순서대로 추가하는 것이 좋은지?

2. **Rolling Contact 고도화 수준**: MuJoCo hinge 근사로 충분한지, 아니면 `<equality><connect>` 기반 실제 rolling 물리가 필요한지?

3. **Isaac Sim export 우선순위**: Phase 3에서 MJCF에 집중하고 Isaac Sim은 별도 Phase로 분리할 것인지?

4. **기존 `tendon`/`rollingContact` JointType**: `types.ts`에 이미 추가된 이 값들을 제거하고 깨끗하게 재설계할 것인지?

5. **Obstacle의 Dynamic 시나리오**: `scenario: 'dynamic'`일 때 장애물이 움직여야 하는지, 아니면 중력 영향만 받는 것인지?
