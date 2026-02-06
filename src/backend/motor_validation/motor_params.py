"""
motor_params.py — 모터 검증 시뮬레이션을 위한 데이터 구조 정의
=================================================================

모든 물리량은 SI 단위를 사용합니다:
  - 토크: Nm
  - 각속도: rad/s
  - 시간: s

이 모듈은 순수 데이터 컨테이너만 포함하며, 어떠한 계산 로직도 없습니다 (SRP).
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from typing import Optional


# ──────────────────────────────────────────────────────────────
# 전역 수치 안정성 상수
# ──────────────────────────────────────────────────────────────
EPSILON: float = 1e-6
"""
수치 안정성을 위한 전역 엡실론 값.
  - 0으로 나누기 방지
  - 부호 함수의 불연속성 제거 (sign function smoothing)
  - 영속도(zero-velocity) 근처 chattering 방지
"""


# ──────────────────────────────────────────────────────────────
# 모터 기본 파라미터
# ──────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class MotorParams:
    """
    모터 데이터시트로부터 추출한 핵심 사양.

    Attributes:
        rated_torque : 정격 토크 (Nm). 연속 운전 가능한 최대 토크.
                       T-N 커브에서 속도=0 일 때의 기준 토크로 사용됩니다.
        peak_torque  : 순간 최대 토크 (Nm). 단시간(수 초) 동안만 허용.
                       일반적으로 rated_torque 의 2~4배.
        max_velocity : 무부하 최대 속도 (rad/s). T-N 커브의 X축 끝점.
        gear_ratio   : 감속비 (Input:Output). 예: 100 → 100:1 감속.
                       모터 출력축 토크 = 모터 토크 × gear_ratio.
    """

    rated_torque: float  # 정격 토크 (Nm) — 연속 운전 기준
    peak_torque: float  # 순간 최대 토크 (Nm) — 단시간 허용
    max_velocity: float  # 무부하 최대 각속도 (rad/s)
    gear_ratio: float  # 감속비 (Input:Output, 예: 100)

    def __post_init__(self) -> None:
        """입력값 유효성 검증 — 물리적으로 불가능한 값 차단."""
        if self.rated_torque <= 0:
            raise ValueError(
                f"[MotorParams] rated_torque는 양수여야 합니다: {self.rated_torque}"
            )
        if self.peak_torque < self.rated_torque:
            raise ValueError(
                f"[MotorParams] peak_torque({self.peak_torque})는 "
                f"rated_torque({self.rated_torque}) 이상이어야 합니다."
            )
        if self.max_velocity <= 0:
            raise ValueError(
                f"[MotorParams] max_velocity는 양수여야 합니다: {self.max_velocity}"
            )
        if self.gear_ratio <= 0:
            raise ValueError(
                f"[MotorParams] gear_ratio는 양수여야 합니다: {self.gear_ratio}"
            )


# ──────────────────────────────────────────────────────────────
# 마찰 파라미터 (5-Parameter Friction Model)
# ──────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class FrictionParams:
    """
    5-파라미터 마찰 모델 사양.

    물리적 마찰 모델:
      T_f = (μ_c + μ_s·exp(-(v/v_st)²) + μ_break) · sign(v) + μ_v · v

    여기서:
      - μ_c       : 쿨롱 마찰 (Coulomb) — 속도에 무관한 일정 저항
      - μ_v       : 점성 마찰 (Viscous) — 속도에 비례하는 저항
      - μ_s       : 스트리벡 마찰 (Stribeck) — 저속에서의 추가 저항
      - μ_break   : 정지 마찰 (Breakaway) — 정지 상태에서의 최대 저항
      - v_st      : 스트리벡 천이 속도 — Stribeck 효과가 사라지는 기준 속도

    Attributes:
        coulomb_friction   : 쿨롱 마찰 토크 (Nm)
        viscous_friction   : 점성 마찰 계수 (Nm/(rad/s))
        stribeck_friction  : 스트리벡 마찰 토크 (Nm)
        breakaway_friction : 정지 마찰 토크 (Nm) — 정지 상태의 한계
        stribeck_velocity  : 스트리벡 천이 속도 (rad/s) — 전환 기준점
        gear_efficiency    : 기어 효율 (0.0 ~ 1.0) — 동력 전달 효율
    """

    coulomb_friction: float  # μ_c : 쿨롱 마찰 토크 (Nm)
    viscous_friction: float  # μ_v : 점성 마찰 계수 (Nm/(rad/s))
    stribeck_friction: float  # μ_s : 스트리벡 마찰 토크 (Nm)
    breakaway_friction: float  # μ_break : 정지 마찰 토크 (Nm)
    stribeck_velocity: float  # v_st : 스트리벡 천이 속도 (rad/s)
    gear_efficiency: float  # η : 기어 효율 (0.0 ~ 1.0)

    def __post_init__(self) -> None:
        """입력값 유효성 검증 — 비물리적 파라미터 차단."""
        if self.coulomb_friction < 0:
            raise ValueError(
                f"[FrictionParams] coulomb_friction은 0 이상이어야 합니다: "
                f"{self.coulomb_friction}"
            )
        if self.viscous_friction < 0:
            raise ValueError(
                f"[FrictionParams] viscous_friction은 0 이상이어야 합니다: "
                f"{self.viscous_friction}"
            )
        if self.stribeck_friction < 0:
            raise ValueError(
                f"[FrictionParams] stribeck_friction은 0 이상이어야 합니다: "
                f"{self.stribeck_friction}"
            )
        if self.breakaway_friction < 0:
            raise ValueError(
                f"[FrictionParams] breakaway_friction은 0 이상이어야 합니다: "
                f"{self.breakaway_friction}"
            )
        if self.stribeck_velocity <= 0:
            raise ValueError(
                f"[FrictionParams] stribeck_velocity는 양수여야 합니다: "
                f"{self.stribeck_velocity}"
            )
        if not (0.0 < self.gear_efficiency <= 1.0):
            raise ValueError(
                f"[FrictionParams] gear_efficiency는 (0.0, 1.0] 범위여야 합니다: "
                f"{self.gear_efficiency}"
            )


# ──────────────────────────────────────────────────────────────
# 시뮬레이션 상태 스냅샷 (로깅용)
# ──────────────────────────────────────────────────────────────
@dataclass
class SimulationState:
    """
    한 시뮬레이션 타임스텝의 전체 상태를 기록하는 스냅샷.

    물리 파이프라인의 각 단계 결과를 모두 저장하여,
    사후 분석(post-analysis) 시 어느 단계에서 문제가 발생했는지 추적 가능.

    Attributes:
        time          : 시뮬레이션 시간 (s)
        q_ref         : 목표 위치 (rad)
        q_act         : 실제 위치 (rad)
        v_act         : 실제 속도 (rad/s)
        tau_command   : PID 출력 토크 (Nm) — Step 1 결과
        tau_limited   : T-N 커브 제한 후 토크 (Nm) — Step 2 결과
        tau_efficiency: 효율 적용 후 토크 (Nm) — Step 3 결과
        tau_friction  : 마찰 토크 손실 (Nm) — Step 4 결과
        tau_final     : MuJoCo에 적용되는 최종 토크 (Nm) — Step 5 결과
    """

    time: float  # 시뮬레이션 시각 (s)
    q_ref: float  # 목표 관절 위치 (rad)
    q_act: float  # 실제 관절 위치 (rad)
    v_act: float  # 실제 관절 속도 (rad/s)
    tau_command: float  # PID 명령 토크 (Nm) — 제어기 출력
    tau_limited: float  # 속도-토크 제한 후 토크 (Nm) — T-N 커브 적용
    tau_efficiency: float  # 기어 효율 적용 후 토크 (Nm) — 구동/피구동 판별
    tau_friction: float  # 마찰 토크 손실 (Nm) — 5-파라미터 모델
    tau_final: float  # 최종 MuJoCo 적용 토크 (Nm) — 액추에이터 입력


# ──────────────────────────────────────────────────────────────
# PID 게인 파라미터 (편의용)
# ──────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class PIDGains:
    """
    PID 제어기 게인 설정.

    Attributes:
        kp       : 비례 게인 (Proportional)
        ki       : 적분 게인 (Integral)
        kd       : 미분 게인 (Derivative)
        i_max    : 적분 와인드업 제한 (Anti-windup clamp)
        d_filter : 미분항 저역통과 필터 계수 (0~1, 1=필터 없음)
    """

    kp: float = 100.0
    ki: float = 0.0
    kd: float = 10.0
    i_max: float = 50.0  # 적분 와인드업 방지 한계값
    d_filter: float = 0.1  # 미분항 LPF 계수 (작을수록 강한 필터)

    def __post_init__(self) -> None:
        if self.kp < 0:
            raise ValueError(f"[PIDGains] kp는 0 이상이어야 합니다: {self.kp}")
        if self.ki < 0:
            raise ValueError(f"[PIDGains] ki는 0 이상이어야 합니다: {self.ki}")
        if self.kd < 0:
            raise ValueError(f"[PIDGains] kd는 0 이상이어야 합니다: {self.kd}")
        if self.i_max <= 0:
            raise ValueError(f"[PIDGains] i_max는 양수여야 합니다: {self.i_max}")
        if not (0.0 < self.d_filter <= 1.0):
            raise ValueError(
                f"[PIDGains] d_filter는 (0.0, 1.0] 범위여야 합니다: {self.d_filter}"
            )
