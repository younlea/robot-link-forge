"""
efficiency_model.py — 효율 모델 및 토크-속도 제한 (순수 수학, Stateless)
=========================================================================

이 모듈은 물리 파이프라인의 Step 2 (T-N Curve 제한)와
Step 3 (방향성 효율 적용)을 구현합니다.

핵심 물리:
  1. T-N 커브: 모터는 고속에서 토크가 선형적으로 감소합니다.
     이는 역기전력(Back-EMF)에 의해 공급 전압이 소진되기 때문입니다.

  2. 효율 모델: 기어박스의 효율은 동력 전달 방향에 따라 다릅니다.
     - 구동(Driving): 모터 → 부하 방향 → 토크가 η배로 감소
     - 피구동(Driven/Braking): 부하 → 모터 방향 → 토크가 1/η배로 증가
       (역구동 시 효율이 저항으로 작용)

참고: 모든 함수는 순수 함수(Pure Function)입니다.
"""

from __future__ import annotations

import numpy as np

from .motor_params import MotorParams, FrictionParams, EPSILON


# ──────────────────────────────────────────────────────────────
# Step 2-1: 속도 기반 가용 토크 계산 (T-N Curve)
# ──────────────────────────────────────────────────────────────
def compute_available_torque(
    velocity: float,
    motor_params: MotorParams,
) -> float:
    """
    T-N 커브(토크-속도 특성 곡선)에 따른 가용 토크를 계산합니다.

    물리적 배경:
      DC/BLDC 모터에서 역기전력(Back-EMF)은 속도에 비례합니다.
      공급 전압이 고정되면, 속도 증가 → 역기전력 증가 → 유효 전압 감소
      → 전류 감소 → 토크 감소.

      이 관계는 선형 근사로 모델링됩니다:
        T_avail = T_rated × (1 - |v| / v_max)

    경계 조건:
      - |v| = 0       → T_avail = T_rated (최대 토크)
      - |v| = v_max   → T_avail = 0 (무부하 속도에서 토크 없음)
      - |v| > v_max   → T_avail = 0 (과속 영역 — 물리적으로 토크 생성 불가)

    Args:
        velocity     : 현재 관절 각속도 (rad/s)
        motor_params : 모터 사양 (MotorParams)

    Returns:
        가용 토크 T_avail (Nm), 항상 ≥ 0
    """
    abs_velocity = abs(velocity)

    # 과속 영역: 모터가 물리적으로 토크를 생성할 수 없는 속도
    if abs_velocity >= motor_params.max_velocity:
        return 0.0

    # T-N 커브 선형 감소: 속도가 증가하면 가용 토크가 선형 감소
    # 이는 역기전력(Back-EMF)에 의한 유효 전압 감소를 반영
    speed_ratio = abs_velocity / motor_params.max_velocity
    available = motor_params.rated_torque * (1.0 - speed_ratio)

    # 수치 안전: 부동소수점 오차로 음수가 되는 것을 방지
    return max(available, 0.0)


# ──────────────────────────────────────────────────────────────
# Step 2: 토크 제한 (Peak + T-N Curve 통합)
# ──────────────────────────────────────────────────────────────
def apply_torque_speed_limit(
    tau_command: float,
    velocity: float,
    motor_params: MotorParams,
) -> float:
    """
    PID 출력 토크에 물리적 한계를 적용합니다.

    파이프라인 Step 2: 토크-속도 제한 & 피크 토크 제한

    적용 순서 (중요 — 순서가 물리적 의미를 가짐):
      1. Peak Limit: 순간 최대 토크 클램핑
         → 모터 전류의 하드웨어 한계 (드라이버 IC 보호)
         → |T_cmd| ≤ T_peak

      2. Speed Limit (T-N Curve): 속도에 따른 가용 토크 클램핑
         → 역기전력에 의한 물리적 한계
         → |T| ≤ T_avail(v)

    이 순서를 지키면 "고속에서 피크 토크를 요구해도 T-N 커브에 의해
    자동으로 줄어드는" 현실적 동작이 재현됩니다.

    Args:
        tau_command  : PID 제어기의 명령 토크 (Nm)
        velocity     : 현재 관절 각속도 (rad/s)
        motor_params : 모터 사양 (MotorParams)

    Returns:
        제한된 토크 T_limited (Nm)
    """
    # ── Step 2-1: Peak Torque Limit (순간 최대 토크 클램핑) ──
    # 모터 드라이버의 전류 제한에 해당
    # 아무리 큰 PID 출력이 나와도 하드웨어가 허용하는 최대 전류를 초과할 수 없음
    tau_clamped = float(
        np.clip(tau_command, -motor_params.peak_torque, motor_params.peak_torque)
    )

    # ── Step 2-2: Speed-Torque Limit (T-N 커브 적용) ──
    # 현재 속도에서의 가용 토크 계산 (역기전력 고려)
    t_available = compute_available_torque(velocity, motor_params)

    # 가용 토크 범위로 최종 클램핑
    # 이 단계에서 고속 영역의 토크 부족이 자연스럽게 반영됨
    tau_limited = float(np.clip(tau_clamped, -t_available, t_available))

    return tau_limited


# ──────────────────────────────────────────────────────────────
# Step 3: 방향성 효율 모델 (Driving vs. Driven)
# ──────────────────────────────────────────────────────────────
def apply_efficiency(
    tau_limited: float,
    velocity: float,
    gear_efficiency: float,
    epsilon: float = EPSILON,
) -> float:
    """
    기어박스 효율을 방향에 따라 적용합니다.

    파이프라인 Step 3: 효율 모델 (방향성)

    기계적 파워 판별:
      P = T_limited × v

      - P > 0 (구동, Driving): 모터가 부하를 밀어냄
        → 기어박스 마찰에 의해 출력 토크 감소
        → T_eff = T_limited × η

      - P < 0 (피구동, Driven/Braking): 부하가 모터를 밀어냄 (역구동)
        → 기어박스 마찰이 역방향 전달을 더 어렵게 만듦
        → T_eff = T_limited / η
        → 효율이 저항으로 작용하여 토크가 증가 (브레이킹 강화)

      - P ≈ 0 (영점 교차): 부드러운 전환을 위해 보간 처리
        → 수치 불연속 방지

    영점 교차 처리 (Zero-Crossing):
      P → 0 에서 η ↔ 1/η 전환이 불연속이면 수치 jitter 발생.
      tanh 기반 블렌딩으로 부드러운 전환을 구현합니다:
        blend = tanh(P / ε)
        T_eff = T_limited × lerp(1/η, η, (blend+1)/2)

    Args:
        tau_limited     : T-N 커브 제한 후 토크 (Nm)
        velocity        : 현재 관절 각속도 (rad/s)
        gear_efficiency : 기어 효율 η (0.0 ~ 1.0)
        epsilon         : 영점 교차 스무딩 파라미터

    Returns:
        효율 적용 후 토크 T_eff (Nm)
    """
    # ── 기계적 파워 계산 ──
    # P = T × ω : 토크와 속도의 부호가 같으면 구동(Driving)
    power = tau_limited * velocity

    # ── 효율 계수 결정 (방향 판별) ──
    # 구동 시: η (토크 감소, 에너지 손실)
    # 피구동 시: 1/η (역구동 저항 증가)
    eta_driving = gear_efficiency  # 구동: 출력 = 입력 × η
    eta_driven = 1.0 / max(gear_efficiency, EPSILON)  # 피구동: 출력 = 입력 / η

    # ── 영점 교차 부드러운 전환 (Smooth Blending) ──
    # tanh로 -1~+1 범위의 블렌딩 계수 생성
    # power_threshold: 전환 영역의 폭을 결정하는 스케일
    # 너무 작으면 급격한 전환 (jitter), 너무 크면 둔감한 응답
    power_threshold = max(abs(tau_limited) * 0.01, epsilon)
    blend_arg = power / power_threshold
    blend_arg_clamped = float(np.clip(blend_arg, -20.0, 20.0))
    blend = float(np.tanh(blend_arg_clamped))

    # blend ∈ [-1, 1] → alpha ∈ [0, 1]
    # alpha = 0 → 피구동 (driven)
    # alpha = 1 → 구동 (driving)
    alpha = (blend + 1.0) / 2.0

    # 선형 보간: 피구동(1/η) ↔ 구동(η)
    effective_eta = alpha * eta_driving + (1.0 - alpha) * eta_driven

    # ── 효율 적용 ──
    tau_efficiency = tau_limited * effective_eta

    return tau_efficiency
