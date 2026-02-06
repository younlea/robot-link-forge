"""
motor_physics.py — 모터 물리 파이프라인 오케스트레이터
======================================================

이 모듈은 물리 파이프라인의 모든 단계를 올바른 순서로 실행하는
**메인 통합 클래스(Orchestrator)** 입니다.

실행 순서 (엄격히 준수):
  Step 1: PID + Feedforward → T_cmd
  Step 2: T-N Curve + Peak Limit → T_limited
  Step 3: 효율 (구동/피구동) → T_efficiency
  Step 4: 5-파라미터 마찰 → T_friction
  Step 5: 최종 토크 = T_efficiency - T_friction → T_final
  Step 6: MuJoCo 액추에이터에 적용 → data.ctrl[joint_id]

이 클래스가 유일하게 다른 모듈들을 import하고 조합하는 곳입니다.
각 물리 모듈(friction_model, efficiency_model)은 이 클래스를 모릅니다.

사용법:
  engine = MotorPhysicsEngine(motor_params, friction_params, pid_gains)
  while simulation_running:
      engine.step(time, q_ref, q_act, v_act, dt, data, joint_id)
"""

from __future__ import annotations

from typing import Optional, Any

from .motor_params import (
    MotorParams,
    FrictionParams,
    PIDGains,
    SimulationState,
    EPSILON,
)
from .pid_controller import PIDController
from .friction_model import compute_friction_torque
from .efficiency_model import apply_torque_speed_limit, apply_efficiency
from .logger import SimulationLogger


class MotorPhysicsEngine:
    """
    모터 물리 파이프라인의 메인 오케스트레이터.

    이 클래스는 개별 물리 모듈을 올바른 순서로 호출하고,
    각 단계의 결과를 로거에 기록합니다.

    설계 원칙:
      - 모든 물리 계산은 하위 모듈에 위임 (SRP)
      - 이 클래스는 흐름 제어(Flow Control)만 담당
      - MuJoCo data 객체와의 인터페이스는 step() 메서드에서만 발생

    Attributes:
        motor_params   : 모터 사양 (불변)
        friction_params: 마찰 사양 (불변)
        pid            : PID 제어기 (상태 보유)
        logger         : 시뮬레이션 로거
    """

    def __init__(
        self,
        motor_params: MotorParams,
        friction_params: FrictionParams,
        pid_gains: Optional[PIDGains] = None,
        joint_name: str = "joint_0",
    ) -> None:
        """
        모터 물리 엔진을 초기화합니다.

        Args:
            motor_params    : 모터 데이터시트 사양 (MotorParams)
            friction_params : 5-파라미터 마찰 사양 (FrictionParams)
            pid_gains       : PID 게인 (선택, 기본값 사용 가능)
            joint_name      : 관절 이름 (로깅에 사용)
        """
        # ── 불변 물리 파라미터 저장 ──
        self._motor_params: MotorParams = motor_params
        self._friction_params: FrictionParams = friction_params

        # ── PID 제어기 생성 ──
        if pid_gains is None:
            pid_gains = PIDGains()  # 기본 게인 사용
        self._pid: PIDController = PIDController(pid_gains)

        # ── 로거 생성 ──
        self._logger: SimulationLogger = SimulationLogger(joint_name=joint_name)

        # ── 마지막 계산 결과 캐시 (디버깅/외부 접근용) ──
        self._last_state: Optional[SimulationState] = None

    # ──────────────────────────────────────────────────────────
    # 프로퍼티 (읽기 전용 접근)
    # ──────────────────────────────────────────────────────────
    @property
    def motor_params(self) -> MotorParams:
        """모터 사양을 반환합니다."""
        return self._motor_params

    @property
    def friction_params(self) -> FrictionParams:
        """마찰 사양을 반환합니다."""
        return self._friction_params

    @property
    def pid(self) -> PIDController:
        """PID 제어기 인스턴스를 반환합니다."""
        return self._pid

    @property
    def logger(self) -> SimulationLogger:
        """시뮬레이션 로거를 반환합니다."""
        return self._logger

    @property
    def last_state(self) -> Optional[SimulationState]:
        """마지막 타임스텝의 상태를 반환합니다."""
        return self._last_state

    # ──────────────────────────────────────────────────────────
    # 리셋
    # ──────────────────────────────────────────────────────────
    def reset(self) -> None:
        """
        엔진의 모든 내부 상태를 초기화합니다.

        새로운 시뮬레이션 시작 시 반드시 호출해야 합니다.
        PID 적분값, 로그 데이터, 캐시가 모두 초기화됩니다.
        """
        self._pid.reset()
        self._logger.clear()
        self._last_state = None

    # ──────────────────────────────────────────────────────────
    # 메인 물리 파이프라인 (핵심)
    # ──────────────────────────────────────────────────────────
    def compute_torque(
        self,
        time: float,
        q_ref: float,
        q_act: float,
        v_act: float,
        dt: float,
        feedforward_torque: float = 0.0,
    ) -> SimulationState:
        """
        물리 파이프라인을 실행하여 최종 토크를 계산합니다.

        이 메서드는 MuJoCo에 직접 토크를 적용하지 않습니다.
        MuJoCo와의 결합을 분리하여 단위 테스트와 독립 사용이 가능하도록 합니다.

        파이프라인 실행 순서:
          Step 1: PID + Feedforward → T_cmd
          Step 2: T-N Curve + Peak Limit → T_limited
          Step 3: 효율 (Driving/Driven) → T_efficiency
          Step 4: 5-파라미터 마찰 → T_friction
          Step 5: T_final = T_efficiency - T_friction

        Args:
            time               : 현재 시뮬레이션 시각 (s)
            q_ref              : 목표 관절 위치 (rad)
            q_act              : 실제 관절 위치 (rad)
            v_act              : 실제 관절 속도 (rad/s)
            dt                 : 시뮬레이션 타임스텝 (s)
            feedforward_torque : 외부 Feedforward 토크 (Nm), 기본값 0

        Returns:
            SimulationState — 파이프라인 각 단계의 결과가 담긴 스냅샷
        """

        # ============================================================
        # Step 1: 명령 토크 생성 (PID + Feedforward)
        # ============================================================
        # T_cmd = Kp·e + Ki·∫e + Kd·(de/dt) + T_ff
        #
        # PID 제어기가 위치 오차를 기반으로 필요한 토크를 계산합니다.
        # Feedforward 토크가 있으면 관성/중력 보상으로 추가됩니다.
        # 이 단계의 출력은 아직 물리적 제한이 적용되지 않은 "이상적" 토크입니다.
        tau_command = self._pid.compute(
            q_ref=q_ref,
            q_act=q_act,
            v_act=v_act,
            dt=dt,
            feedforward_torque=feedforward_torque,
        )

        # ============================================================
        # Step 2: 토크-속도 제한 (T-N Curve + Peak Limit)
        # ============================================================
        # 모터의 물리적 한계를 적용합니다.
        #
        # 1) Peak Limit: |T_cmd| ≤ T_peak (하드웨어 전류 제한)
        # 2) Speed Limit: T_avail = T_rated × (1 - |v|/v_max)
        #    → 역기전력(Back-EMF)에 의한 고속 토크 감소
        #
        # 이 단계 후 토크는 현재 속도에서 모터가 실제로 생성할 수 있는
        # 최대 토크 이내로 제한됩니다.
        tau_limited = apply_torque_speed_limit(
            tau_command=tau_command,
            velocity=v_act,
            motor_params=self._motor_params,
        )

        # ============================================================
        # Step 3: 기어 효율 적용 (구동/피구동 방향 판별)
        # ============================================================
        # P = T_limited × v 로 기계적 파워의 부호를 판별합니다.
        #
        # P > 0 (구동, Driving): 모터 → 부하
        #   → T_eff = T_limited × η (효율에 의한 손실)
        #
        # P < 0 (피구동, Driven): 부하 → 모터 (역구동)
        #   → T_eff = T_limited / η (효율이 저항으로 작용)
        #
        # P ≈ 0: tanh 기반 블렌딩으로 부드러운 전환
        tau_efficiency = apply_efficiency(
            tau_limited=tau_limited,
            velocity=v_act,
            gear_efficiency=self._friction_params.gear_efficiency,
        )

        # ============================================================
        # Step 4: 5-파라미터 마찰 토크 계산
        # ============================================================
        # T_f = (μ_c + μ_s·exp(-(v/v_st)²) + μ_break) · sign(v) + μ_v · v
        #
        # 마찰 토크는 **항상 운동을 방해하는 방향**으로 작용합니다.
        # sign(v)는 tanh 기반 수치 안정 근사를 사용합니다.
        #
        # 각 성분:
        #   - 쿨롱(μ_c): 일정 저항
        #   - 스트리벡(μ_s): 저속 추가 저항 (윤활 불량)
        #   - 정지마찰(μ_break): 정지 돌파 저항
        #   - 점성(μ_v): 속도 비례 저항 (유체역학)
        tau_friction = compute_friction_torque(
            velocity=v_act,
            friction_params=self._friction_params,
        )

        # ============================================================
        # Step 5: 최종 토크 계산
        # ============================================================
        # T_final = T_efficiency - T_friction
        #
        # 효율 적용 후 토크에서 마찰 손실을 빼서 최종 토크를 산출합니다.
        # 이 값이 MuJoCo 액추에이터에 실제로 적용되는 토크입니다.
        tau_final = tau_efficiency - tau_friction

        # ============================================================
        # 상태 스냅샷 생성 및 로깅
        # ============================================================
        state = SimulationState(
            time=time,
            q_ref=q_ref,
            q_act=q_act,
            v_act=v_act,
            tau_command=tau_command,
            tau_limited=tau_limited,
            tau_efficiency=tau_efficiency,
            tau_friction=tau_friction,
            tau_final=tau_final,
        )

        # 로거에 기록 (사후 분석용)
        self._logger.log(state)

        # 마지막 상태 캐시 (외부 접근용)
        self._last_state = state

        return state

    # ──────────────────────────────────────────────────────────
    # MuJoCo 통합 메서드 (Step 6 포함)
    # ──────────────────────────────────────────────────────────
    def step(
        self,
        time: float,
        q_ref: float,
        q_act: float,
        v_act: float,
        dt: float,
        mj_data: Any,
        actuator_id: int,
        feedforward_torque: float = 0.0,
    ) -> SimulationState:
        """
        물리 파이프라인을 실행하고 MuJoCo 액추에이터에 직접 적용합니다.

        이 메서드는 compute_torque()를 호출한 후,
        Step 6으로서 결과 토크를 MuJoCo data.ctrl에 기록합니다.

        주의:
          - mj_data는 MuJoCo의 mujoco.MjData 객체여야 합니다.
          - actuator_id는 data.ctrl 배열의 인덱스입니다.
          - 이 메서드 호출 후 mujoco.mj_step()을 별도로 호출해야 합니다.

        Args:
            time               : 현재 시뮬레이션 시각 (s)
            q_ref              : 목표 관절 위치 (rad)
            q_act              : 실제 관절 위치 (rad)
            v_act              : 실제 관절 속도 (rad/s)
            dt                 : 시뮬레이션 타임스텝 (s)
            mj_data            : MuJoCo MjData 객체
            actuator_id        : 액추에이터 인덱스 (data.ctrl의 인덱스)
            feedforward_torque : 외부 Feedforward 토크 (Nm)

        Returns:
            SimulationState — 파이프라인 각 단계의 결과 스냅샷
        """
        # ── Step 1~5: 물리 파이프라인 실행 ──
        state = self.compute_torque(
            time=time,
            q_ref=q_ref,
            q_act=q_act,
            v_act=v_act,
            dt=dt,
            feedforward_torque=feedforward_torque,
        )

        # ============================================================
        # Step 6: MuJoCo 액추에이터에 최종 토크 적용
        # ============================================================
        # data.ctrl[actuator_id]에 토크를 기록합니다.
        # MuJoCo는 이 값을 다음 mj_step() 호출 시 사용합니다.
        #
        # 주의: MuJoCo의 actuator type이 'motor'(토크 제어)로 설정되어 있어야 합니다.
        # position/velocity actuator로 설정된 경우 ctrl 값의 의미가 달라집니다.
        mj_data.ctrl[actuator_id] = state.tau_final

        return state

    # ──────────────────────────────────────────────────────────
    # 편의 메서드
    # ──────────────────────────────────────────────────────────
    def export_log(self, filepath: str = "motor_validation_log.csv") -> str:
        """로그를 CSV로 내보냅니다."""
        return self._logger.to_csv(filepath)

    def print_report(self) -> None:
        """분석 리포트를 콘솔에 출력합니다."""
        self._logger.print_analysis(self._motor_params)

    def get_report(self) -> dict:
        """분석 리포트를 딕셔너리로 반환합니다."""
        return self._logger.get_analysis(self._motor_params)
