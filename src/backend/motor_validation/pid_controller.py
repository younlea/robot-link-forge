"""
pid_controller.py — PID 제어기 (Anti-Windup, 미분 필터 포함)
=============================================================

이 모듈은 위치 기반 PID 제어기를 구현합니다.
Feedforward 토크를 외부에서 주입할 수 있으며,
적분 와인드업 방지와 미분항 저역통과 필터를 포함합니다.

파이프라인 Step 1:
  T_cmd = PID(e, ė) + T_ff

여기서:
  - e = q_ref - q_act (위치 오차)
  - ė = dq/dt (속도, MuJoCo에서 직접 읽음)
  - T_ff = 외부 Feedforward 토크 (옵션)

PID 클래스는 **유일하게 내부 상태를 가지는** 모듈입니다.
(적분항, 이전 오차, 필터된 미분값)
"""

from __future__ import annotations

import numpy as np

from .motor_params import PIDGains, EPSILON


class PIDController:
    """
    Anti-Windup 및 미분 필터를 갖춘 PID 제어기.

    특징:
      1. 적분 와인드업 방지 (Anti-Windup Clamping)
         - 적분항이 i_max를 초과하지 않도록 클램핑
         - 액추에이터 포화 시 적분항 폭주 방지

      2. 미분항 저역통과 필터 (Derivative Low-Pass Filter)
         - 센서 노이즈에 의한 미분 스파이크 억제
         - d_filter 계수로 필터 강도 조절 (0 = 최대 필터, 1 = 필터 없음)

      3. Feedforward 토크 주입
         - 궤적 계획기(Trajectory Planner)로부터의 FF 토크 추가
         - 관성 보상, 중력 보상 등에 활용

    내부 상태:
      - _integral      : 적분항 누적값
      - _prev_error    : 이전 시간 스텝의 오차 (미분 계산용)
      - _filtered_deriv: 필터링된 미분값
      - _initialized   : 최초 호출 여부 플래그
    """

    def __init__(self, gains: PIDGains) -> None:
        """
        PID 제어기를 초기화합니다.

        Args:
            gains : PID 게인 설정 (PIDGains dataclass)
        """
        self._gains: PIDGains = gains

        # ── 내부 상태 초기화 ──
        self._integral: float = 0.0  # 적분항 누적값 (Nm·s)
        self._prev_error: float = 0.0  # 이전 타임스텝 오차 (rad)
        self._filtered_deriv: float = 0.0  # LPF 적용된 미분값 (rad/s)
        self._initialized: bool = False  # 최초 호출 감지 플래그

    def reset(self) -> None:
        """
        PID 내부 상태를 초기화합니다.

        새로운 궤적 시작 시 또는 시뮬레이션 리셋 시 호출해야 합니다.
        이전 궤적의 적분값이 새 궤적에 영향을 주는 것을 방지합니다.
        """
        self._integral = 0.0
        self._prev_error = 0.0
        self._filtered_deriv = 0.0
        self._initialized = False

    def compute(
        self,
        q_ref: float,
        q_act: float,
        v_act: float,
        dt: float,
        feedforward_torque: float = 0.0,
    ) -> float:
        """
        PID 제어 토크를 계산합니다.

        파이프라인 Step 1: 명령 토크 생성
          T_cmd = Kp·e + Ki·∫e·dt + Kd·(de/dt)_filtered + T_ff

        미분항 계산 방식:
          - MuJoCo에서 직접 읽은 v_act(실제 속도)를 사용하지 않고,
            오차의 유한 차분(finite difference)을 사용합니다.
          - 이유: v_act는 관절 속도이지 오차 변화율이 아님.
            목표 속도가 0이 아닌 경우 v_act를 미분항으로 쓰면 오류 발생.
          - de/dt ≈ (e[k] - e[k-1]) / dt
          - 노이즈 억제를 위해 LPF를 적용합니다.

        Args:
            q_ref              : 목표 위치 (rad)
            q_act              : 실제 위치 (rad)
            v_act              : 실제 속도 (rad/s) — 현재 미사용, 로깅용 예비
            dt                 : 시뮬레이션 타임스텝 (s)
            feedforward_torque : 외부 Feedforward 토크 (Nm), 기본값 0

        Returns:
            명령 토크 T_cmd (Nm)
        """
        # ── dt 보호: 0 또는 음수 타임스텝 방지 ──
        dt_safe = max(dt, EPSILON)

        # ── 위치 오차 계산 ──
        error = q_ref - q_act

        # ── 비례항 (P) ──
        p_term = self._gains.kp * error

        # ── 적분항 (I) + Anti-Windup ──
        # 오일러 적분으로 오차 누적
        self._integral += error * dt_safe

        # 와인드업 방지: 적분값을 [-i_max, +i_max] 범위로 클램핑
        # 이렇게 하면 액추에이터가 포화 상태에서 적분항이 계속 증가하는 것을 방지
        self._integral = float(
            np.clip(self._integral, -self._gains.i_max, self._gains.i_max)
        )

        i_term = self._gains.ki * self._integral

        # ── 미분항 (D) + 저역통과 필터 ──
        if not self._initialized:
            # 최초 호출: 이전 오차가 없으므로 미분항 = 0
            # 급격한 초기 스파이크 방지
            raw_deriv = 0.0
            self._initialized = True
        else:
            # 유한 차분으로 오차 변화율 계산
            raw_deriv = (error - self._prev_error) / dt_safe

        # 1차 저역통과 필터 (Exponential Moving Average)
        # d_filter가 작을수록 강한 필터링 (이전 값에 더 의존)
        # d_filter = 1.0 이면 필터 없음 (raw 값 그대로)
        alpha = self._gains.d_filter
        self._filtered_deriv = alpha * raw_deriv + (1.0 - alpha) * self._filtered_deriv

        d_term = self._gains.kd * self._filtered_deriv

        # 이전 오차 저장 (다음 스텝의 미분 계산에 사용)
        self._prev_error = error

        # ── 최종 PID 출력 + Feedforward ──
        # T_cmd = P + I + D + T_ff
        tau_command = p_term + i_term + d_term + feedforward_torque

        return tau_command

    @property
    def gains(self) -> PIDGains:
        """현재 PID 게인을 반환합니다 (읽기 전용)."""
        return self._gains

    @gains.setter
    def gains(self, new_gains: PIDGains) -> None:
        """
        PID 게인을 런타임에 변경합니다.
        게인 변경 시 적분항은 리셋하지 않습니다 (Bumpless Transfer).
        """
        self._gains = new_gains
