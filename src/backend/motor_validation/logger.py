"""
logger.py — 시뮬레이션 상태 로거 (CSV 내보내기 포함)
=====================================================

이 모듈은 물리 파이프라인의 각 단계별 결과를 수집하고,
사후 분석을 위해 CSV 파일로 내보내는 기능을 제공합니다.

로깅 항목 (매 타임스텝):
  time, q_ref, q_act, v_act,
  tau_command, tau_limited, tau_efficiency, tau_friction, tau_final

분석 기능:
  - 토크 포화 감지 (Torque Saturation)
  - RMS 토크 계산 (발열 검증)
  - 위치 추종 오차 통계
  - T-N 커브 동작 영역 분석
"""

from __future__ import annotations

import csv
import os
import numpy as np
from typing import List, Dict, Any, Optional

from .motor_params import SimulationState, MotorParams


class SimulationLogger:
    """
    시뮬레이션 상태를 타임스텝별로 기록하는 로거.

    사용 패턴:
      1. 시뮬레이션 루프에서 매 스텝마다 log() 호출
      2. 시뮬레이션 종료 후 to_csv() 또는 get_analysis() 호출
      3. 새 시뮬레이션 시작 시 clear() 호출

    메모리 관리:
      - 내부적으로 SimulationState 리스트를 유지
      - 대규모 시뮬레이션(>1M 스텝) 시 메모리 주의
    """

    def __init__(self, joint_name: str = "joint_0") -> None:
        """
        로거를 초기화합니다.

        Args:
            joint_name : 로깅 대상 관절 이름 (CSV 헤더/파일명에 사용)
        """
        self._joint_name: str = joint_name
        self._states: List[SimulationState] = []

    def log(self, state: SimulationState) -> None:
        """
        한 타임스텝의 시뮬레이션 상태를 기록합니다.

        Args:
            state : 현재 타임스텝의 전체 상태 스냅샷 (SimulationState)
        """
        self._states.append(state)

    def clear(self) -> None:
        """기록된 모든 상태를 삭제합니다."""
        self._states.clear()

    @property
    def states(self) -> List[SimulationState]:
        """기록된 전체 상태 리스트를 반환합니다 (읽기 전용 복사본)."""
        return list(self._states)

    @property
    def num_steps(self) -> int:
        """기록된 타임스텝 수를 반환합니다."""
        return len(self._states)

    # ──────────────────────────────────────────────────────────
    # CSV 내보내기
    # ──────────────────────────────────────────────────────────
    def to_csv(self, filepath: str) -> str:
        """
        기록된 상태를 CSV 파일로 내보냅니다.

        CSV 컬럼:
          time, q_ref, q_act, v_act,
          tau_command, tau_limited, tau_efficiency, tau_friction, tau_final

        Args:
            filepath : 출력 CSV 파일 경로

        Returns:
            생성된 파일의 절대 경로
        """
        # 디렉토리가 없으면 생성
        dirpath = os.path.dirname(filepath)
        if dirpath:
            os.makedirs(dirpath, exist_ok=True)

        headers = [
            "time",
            "q_ref",
            "q_act",
            "v_act",
            "tau_command",
            "tau_limited",
            "tau_efficiency",
            "tau_friction",
            "tau_final",
        ]

        with open(filepath, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            for s in self._states:
                writer.writerow(
                    [
                        f"{s.time:.6f}",
                        f"{s.q_ref:.6f}",
                        f"{s.q_act:.6f}",
                        f"{s.v_act:.6f}",
                        f"{s.tau_command:.6f}",
                        f"{s.tau_limited:.6f}",
                        f"{s.tau_efficiency:.6f}",
                        f"{s.tau_friction:.6f}",
                        f"{s.tau_final:.6f}",
                    ]
                )

        return os.path.abspath(filepath)

    # ──────────────────────────────────────────────────────────
    # NumPy 배열 변환 (플로팅/분석용)
    # ──────────────────────────────────────────────────────────
    def to_numpy(self) -> Dict[str, np.ndarray]:
        """
        기록된 상태를 NumPy 배열 딕셔너리로 변환합니다.

        Returns:
            각 필드명을 키로 하는 1D numpy 배열 딕셔너리.
            예: {"time": array([...]), "q_ref": array([...]), ...}
        """
        if not self._states:
            return {
                key: np.array([], dtype=np.float64)
                for key in [
                    "time",
                    "q_ref",
                    "q_act",
                    "v_act",
                    "tau_command",
                    "tau_limited",
                    "tau_efficiency",
                    "tau_friction",
                    "tau_final",
                ]
            }

        return {
            "time": np.array([s.time for s in self._states], dtype=np.float64),
            "q_ref": np.array([s.q_ref for s in self._states], dtype=np.float64),
            "q_act": np.array([s.q_act for s in self._states], dtype=np.float64),
            "v_act": np.array([s.v_act for s in self._states], dtype=np.float64),
            "tau_command": np.array(
                [s.tau_command for s in self._states], dtype=np.float64
            ),
            "tau_limited": np.array(
                [s.tau_limited for s in self._states], dtype=np.float64
            ),
            "tau_efficiency": np.array(
                [s.tau_efficiency for s in self._states], dtype=np.float64
            ),
            "tau_friction": np.array(
                [s.tau_friction for s in self._states], dtype=np.float64
            ),
            "tau_final": np.array(
                [s.tau_final for s in self._states], dtype=np.float64
            ),
        }

    # ──────────────────────────────────────────────────────────
    # 자동 분석 리포트
    # ──────────────────────────────────────────────────────────
    def get_analysis(
        self, motor_params: Optional[MotorParams] = None
    ) -> Dict[str, Any]:
        """
        기록된 데이터에 대한 자동 분석 리포트를 생성합니다.

        분석 항목:
          1. 토크 포화 비율 — 전체 스텝 중 T-N 커브에 의해 클램핑된 비율
          2. RMS 토크 — 발열 검증 기준 (정격 토크 대비)
          3. 최대 위치 오차 — 궤적 추종 품질
          4. 평균/최대 속도 — 속도 한계 도달 여부
          5. Pass/Fail 판정 — motor_params 제공 시

        Args:
            motor_params : 모터 사양 (선택). 제공 시 Pass/Fail 판정 수행.

        Returns:
            분석 결과 딕셔너리
        """
        if not self._states:
            return {"error": "로그 데이터가 없습니다."}

        data = self.to_numpy()

        # ── 기본 통계 ──
        position_error = np.abs(data["q_ref"] - data["q_act"])
        abs_torque_final = np.abs(data["tau_final"])

        analysis: Dict[str, Any] = {
            "joint_name": self._joint_name,
            "total_steps": self.num_steps,
            "duration_s": float(data["time"][-1] - data["time"][0]),
            # 위치 추종 오차
            "max_position_error_rad": float(np.max(position_error)),
            "mean_position_error_rad": float(np.mean(position_error)),
            "rms_position_error_rad": float(np.sqrt(np.mean(position_error**2))),
            # 토크 통계
            "max_torque_Nm": float(np.max(abs_torque_final)),
            "mean_torque_Nm": float(np.mean(abs_torque_final)),
            "rms_torque_Nm": float(np.sqrt(np.mean(data["tau_final"] ** 2))),
            # 속도 통계
            "max_velocity_rad_s": float(np.max(np.abs(data["v_act"]))),
            "mean_velocity_rad_s": float(np.mean(np.abs(data["v_act"]))),
            # 토크 포화 분석 (command vs limited 비교)
            "saturation_ratio": float(
                np.mean(np.abs(data["tau_command"] - data["tau_limited"]) > 1e-4)
            ),
            # 마찰 손실
            "mean_friction_loss_Nm": float(np.mean(np.abs(data["tau_friction"]))),
        }

        # ── Pass/Fail 판정 (모터 사양 제공 시) ──
        if motor_params is not None:
            rms_torque = analysis["rms_torque_Nm"]
            rated = motor_params.rated_torque

            # RMS 토크가 정격 토크 이하면 발열 안전
            analysis["thermal_pass"] = bool(rms_torque <= rated)
            analysis["thermal_margin_percent"] = float(
                (1.0 - rms_torque / max(rated, 1e-9)) * 100.0
            )

            # 토크 포화 비율이 5% 이하면 토크 충분
            analysis["torque_pass"] = bool(analysis["saturation_ratio"] <= 0.05)

            # 최대 위치 오차가 0.1 rad (≈5.7°) 이하면 추종 양호
            analysis["tracking_pass"] = bool(analysis["max_position_error_rad"] <= 0.1)

            # 종합 판정
            analysis["overall_pass"] = bool(
                analysis["thermal_pass"]
                and analysis["torque_pass"]
                and analysis["tracking_pass"]
            )

        return analysis

    def print_analysis(self, motor_params: Optional[MotorParams] = None) -> None:
        """분석 결과를 콘솔에 출력합니다."""
        report = self.get_analysis(motor_params)

        print("\n" + "=" * 60)
        print(f"  모터 검증 분석 리포트 — [{report.get('joint_name', '?')}]")
        print("=" * 60)
        print(f"  시뮬레이션 시간 : {report.get('duration_s', 0):.3f} s")
        print(f"  총 스텝 수      : {report.get('total_steps', 0)}")
        print("-" * 60)
        print(f"  [위치 추종]")
        print(f"    최대 오차     : {report.get('max_position_error_rad', 0):.4f} rad")
        print(f"    평균 오차     : {report.get('mean_position_error_rad', 0):.4f} rad")
        print(f"    RMS 오차      : {report.get('rms_position_error_rad', 0):.4f} rad")
        print("-" * 60)
        print(f"  [토크]")
        print(f"    최대 토크     : {report.get('max_torque_Nm', 0):.4f} Nm")
        print(f"    RMS 토크      : {report.get('rms_torque_Nm', 0):.4f} Nm")
        print(f"    포화 비율     : {report.get('saturation_ratio', 0) * 100:.1f} %")
        print(f"    평균 마찰 손실: {report.get('mean_friction_loss_Nm', 0):.4f} Nm")
        print("-" * 60)
        print(f"  [속도]")
        print(f"    최대 속도     : {report.get('max_velocity_rad_s', 0):.4f} rad/s")

        if "overall_pass" in report:
            print("-" * 60)
            print(f"  [Pass/Fail 판정]")
            _pass_str = lambda b: "✅ PASS" if b else "❌ FAIL"
            print(
                f"    발열 안전     : {_pass_str(report['thermal_pass'])} "
                f"(마진: {report['thermal_margin_percent']:.1f}%)"
            )
            print(f"    토크 충분     : {_pass_str(report['torque_pass'])}")
            print(f"    추종 양호     : {_pass_str(report['tracking_pass'])}")
            print(f"    ────────────")
            print(f"    종합 판정     : {_pass_str(report['overall_pass'])}")

        print("=" * 60 + "\n")
