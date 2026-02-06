"""
friction_model.py — 5-파라미터 마찰 모델 (순수 수학, Stateless)
================================================================

이 모듈은 Stribeck + Coulomb + Viscous + Breakaway 마찰을 포함하는
5-파라미터 마찰 모델을 구현합니다.

물리적 배경:
  실제 기어박스/베어링에서의 마찰은 단순한 쿨롱 마찰만으로는 설명되지 않습니다.
  저속 영역에서는 Stribeck 효과에 의해 마찰이 오히려 증가하며,
  고속에서는 점성 마찰이 지배적입니다.

마찰 모델 공식:
  T_stribeck = μ_s · exp(-(v / v_st)²)
  T_f = (μ_c + T_stribeck + μ_break) · sign(v) + μ_v · v

  여기서 sign(v) ≈ tanh(v / ε) 으로 수치적으로 안정한 근사를 사용합니다.

참고: 이 함수들은 모두 **순수 함수(Pure Function)** 입니다.
  - 내부 상태를 변경하지 않습니다.
  - 동일 입력 → 동일 출력 (결정론적, Deterministic)
"""

from __future__ import annotations

import numpy as np

from .motor_params import FrictionParams, EPSILON


# ──────────────────────────────────────────────────────────────
# 수치 안정 부호 함수 (Numerically Stable Sign Function)
# ──────────────────────────────────────────────────────────────
def smooth_sign(velocity: float, epsilon: float = EPSILON) -> float:
    """
    수치적으로 안정한 부호 함수.

    tanh 기반 근사를 사용하여 v=0 근처에서의 불연속성을 제거합니다.
    이는 MuJoCo 솔버의 chattering(진동) 현상을 방지하는 핵심 기법입니다.

    수학적 정의:
      sign(v) ≈ tanh(v / ε)

    tanh 선택 이유 (vs v/(|v|+ε)):
      - tanh는 C∞ 급 매끄러운 함수 (무한 미분 가능)
      - 미분이 연속이므로 수치 적분기에 더 친화적
      - 포화 특성이 자연스러워 Stribeck 영역에서의 전환이 부드러움
      - v/(|v|+ε) 방식은 ε 근처에서 기울기 불연속이 존재

    Args:
        velocity : 관절 각속도 (rad/s)
        epsilon  : 스무딩 파라미터 — 작을수록 이상적 sign에 근접
                   (기본값: 1e-6)

    Returns:
        -1.0 ~ +1.0 범위의 부드러운 부호값
    """
    # tanh의 인자가 너무 커지면 오버플로우 방지를 위해 클램핑
    # tanh(20) ≈ 1.0 이므로 실질적으로 포화
    arg = velocity / epsilon
    arg_clamped = np.clip(arg, -20.0, 20.0)
    return float(np.tanh(arg_clamped))


# ──────────────────────────────────────────────────────────────
# Stribeck 마찰 성분 계산
# ──────────────────────────────────────────────────────────────
def compute_stribeck_component(
    velocity: float,
    stribeck_friction: float,
    stribeck_velocity: float,
) -> float:
    """
    Stribeck 마찰 성분을 계산합니다.

    Stribeck 효과는 저속 영역에서 윤활막이 완전히 형성되지 않아
    마찰이 증가하는 현상을 모델링합니다.
    속도가 증가하면 지수적으로 감소하여 고속에서는 무시됩니다.

    수학적 정의:
      T_stribeck = μ_s · exp(-(v / v_st)²)

    여기서:
      - μ_s  : 스트리벡 마찰 계수 (Nm) — 저속에서의 추가 저항 크기
      - v_st : 스트리벡 천이 속도 (rad/s) — 효과가 사라지는 기준점
      - v    : 현재 속도 (rad/s)

    Args:
        velocity          : 현재 관절 각속도 (rad/s)
        stribeck_friction : 스트리벡 마찰 토크 μ_s (Nm)
        stribeck_velocity : 스트리벡 천이 속도 v_st (rad/s)

    Returns:
        Stribeck 마찰 토크 성분 (Nm), 항상 ≥ 0
    """
    # 속도/천이속도 비율의 제곱 — 지수 감쇠의 핵심
    # stribeck_velocity는 FrictionParams에서 양수 검증 완료
    velocity_ratio = velocity / stribeck_velocity
    exponent = -(velocity_ratio**2)

    # 지수 인자가 너무 작아지면 0으로 처리 (불필요한 연산 방지)
    # exp(-500) ≈ 7e-218 → 사실상 0
    if exponent < -500.0:
        return 0.0

    return stribeck_friction * float(np.exp(exponent))


# ──────────────────────────────────────────────────────────────
# 5-파라미터 마찰 토크 계산 (메인 함수)
# ──────────────────────────────────────────────────────────────
def compute_friction_torque(
    velocity: float,
    friction_params: FrictionParams,
    epsilon: float = EPSILON,
) -> float:
    """
    5-파라미터 마찰 모델에 의한 마찰 토크를 계산합니다.

    이 함수는 물리 파이프라인의 **Step 4** 에 해당합니다.
    마찰 토크는 **항상 운동을 방해하는 방향**으로 작용합니다.

    전체 마찰 모델:
      T_stribeck = μ_s · exp(-(v / v_st)²)
      T_f = (μ_c + T_stribeck + μ_break) · sign(v) + μ_v · v

    각 성분의 물리적 의미:
      1. μ_c · sign(v)         — 쿨롱 마찰: 속도에 무관한 일정 저항력
      2. T_stribeck · sign(v)  — 스트리벡: 저속에서 윤활 불량으로 인한 추가 저항
      3. μ_break · sign(v)     — 정지 마찰: 정지 상태에서의 돌파 저항
      4. μ_v · v               — 점성 마찰: 속도에 비례하는 유체역학적 저항

    부호 규칙:
      - sign(v) 성분들: 운동 방향과 반대로 작용 (에너지 소산)
      - μ_v · v: 점성항은 자체적으로 속도와 같은 부호를 가짐
        → 결과적으로 마찰 토크는 운동을 감속시키는 방향

    Args:
        velocity        : 현재 관절 각속도 (rad/s)
        friction_params : 5-파라미터 마찰 사양 (FrictionParams)
        epsilon         : sign 함수 스무딩 파라미터 (기본값: 1e-6)

    Returns:
        마찰 토크 T_f (Nm) — 이 값을 효율 적용 후 토크에서 **빼야** 합니다.
    """
    # ── 1단계: 수치 안정 부호 함수 계산 ──
    # v=0 근처에서 chattering 방지를 위해 tanh 기반 smooth sign 사용
    sign_v = smooth_sign(velocity, epsilon)

    # ── 2단계: Stribeck 마찰 성분 계산 ──
    # 저속 영역에서의 추가 마찰 (속도 증가 시 지수 감소)
    t_stribeck = compute_stribeck_component(
        velocity=velocity,
        stribeck_friction=friction_params.stribeck_friction,
        stribeck_velocity=friction_params.stribeck_velocity,
    )

    # ── 3단계: 방향성 마찰 성분 합산 ──
    # 쿨롱 + 스트리벡 + 정지마찰 → 모두 운동 반대 방향으로 작용
    directional_component = (
        friction_params.coulomb_friction
        + t_stribeck
        + friction_params.breakaway_friction
    ) * sign_v

    # ── 4단계: 점성 마찰 (속도 비례) ──
    # 유체/그리스에 의한 저항 — 속도에 비례하여 선형 증가
    viscous_component = friction_params.viscous_friction * velocity

    # ── 5단계: 전체 마찰 토크 합산 ──
    friction_torque = directional_component + viscous_component

    return friction_torque
