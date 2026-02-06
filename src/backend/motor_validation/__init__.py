"""
motor_validation — 고급 모터 검증 레이어 (Function 3)
=====================================================

MuJoCo 기반 시뮬레이터를 위한 물리적으로 정확한 모터 다이나믹스 모듈.
모터 데이터시트 파라미터를 기반으로 궤적 실행 가능성을 검증합니다.

물리 파이프라인:
  Input → PID + FF → T-N Curve Limit → Efficiency → 5-Param Friction → MuJoCo

모듈 구조:
  - motor_params.py     : 데이터 구조 (MotorParams, FrictionParams, SimulationState)
  - friction_model.py   : 5-파라미터 마찰 모델 (Stateless)
  - efficiency_model.py : 효율 및 T-N 커브 제한 (Stateless)
  - pid_controller.py   : PID 제어기 (Anti-windup, 미분 필터)
  - motor_physics.py    : 파이프라인 오케스트레이터
  - logger.py           : 시뮬레이션 로거 (CSV, 분석)

사용 예시:
    from motor_validation import MotorPhysicsEngine, MotorParams, FrictionParams

    motor = MotorParams(rated_torque=1.5, peak_torque=4.5, max_velocity=5.2, gear_ratio=100)
    friction = FrictionParams(
        coulomb_friction=0.1, viscous_friction=0.01, stribeck_friction=0.05,
        breakaway_friction=0.15, stribeck_velocity=0.1, gear_efficiency=0.8,
    )
    engine = MotorPhysicsEngine(motor, friction)

    # MuJoCo 루프 내에서:
    state = engine.step(time, q_ref, q_act, v_act, dt, mj_data, actuator_id)

참고 문서:
  https://younlea.github.io/robot/mujoco-simul/
"""

from .motor_params import (
    MotorParams,
    FrictionParams,
    SimulationState,
    PIDGains,
    EPSILON,
)
from .friction_model import (
    compute_friction_torque,
    compute_stribeck_component,
    smooth_sign,
)
from .efficiency_model import (
    apply_torque_speed_limit,
    apply_efficiency,
    compute_available_torque,
)
from .pid_controller import PIDController
from .motor_physics import MotorPhysicsEngine
from .logger import SimulationLogger

__all__ = [
    # 데이터 구조
    "MotorParams",
    "FrictionParams",
    "SimulationState",
    "PIDGains",
    "EPSILON",
    # 마찰 모델
    "compute_friction_torque",
    "compute_stribeck_component",
    "smooth_sign",
    # 효율 모델
    "apply_torque_speed_limit",
    "apply_efficiency",
    "compute_available_torque",
    # 제어기
    "PIDController",
    # 오케스트레이터
    "MotorPhysicsEngine",
    # 로거
    "SimulationLogger",
]
