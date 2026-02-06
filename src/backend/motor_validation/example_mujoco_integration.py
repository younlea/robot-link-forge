"""
example_mujoco_integration.py — MuJoCo 시뮬레이션 루프 통합 예제
================================================================

이 스크립트는 motor_validation 모듈을 MuJoCo 시뮬레이션 루프에
통합하는 방법을 보여줍니다.

실행 전 필요 사항:
  pip install mujoco numpy matplotlib

사용법:
  python example_mujoco_integration.py

출력물:
  1. motor_validation_log.csv — 전체 파이프라인 로그
  2. 콘솔 분석 리포트 (Pass/Fail 판정)
  3. matplotlib 그래프 (토크, 위치 추종, T-N 커브)
"""

from __future__ import annotations

import sys
import os

# motor_validation 패키지를 import할 수 있도록 경로 설정
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

# ── motor_validation 패키지 import ──
from motor_validation import (
    MotorParams,
    FrictionParams,
    PIDGains,
    MotorPhysicsEngine,
)


def run_validation_without_mujoco() -> None:
    """
    MuJoCo 없이 motor_validation 파이프라인만 테스트하는 예제.

    MuJoCo가 설치되지 않은 환경에서도 물리 파이프라인의
    정확성을 검증할 수 있습니다.
    """

    # ──────────────────────────────────────────────────────────
    # 1. 모터 사양 정의 (데이터시트 기반)
    # ──────────────────────────────────────────────────────────
    # 예시: Dynamixel XM430-W350 (감속비 포함 출력축 기준)
    motor_params = MotorParams(
        rated_torque=1.5,  # 정격 토크 (Nm) — 연속 운전 한계
        peak_torque=4.5,  # 순간 최대 토크 (Nm) — 짧은 시간만 허용
        max_velocity=5.2,  # 무부하 최대 속도 (rad/s)
        gear_ratio=100.0,  # 감속비 100:1
    )

    # ──────────────────────────────────────────────────────────
    # 2. 마찰 파라미터 정의 (실험/추정 기반)
    # ──────────────────────────────────────────────────────────
    friction_params = FrictionParams(
        coulomb_friction=0.1,  # 쿨롱 마찰 (Nm) — 정격 토크의 ~7%
        viscous_friction=0.01,  # 점성 마찰 (Nm/(rad/s))
        stribeck_friction=0.05,  # 스트리벡 마찰 (Nm) — 저속 추가 저항
        breakaway_friction=0.15,  # 정지 마찰 (Nm) — 정격의 ~10%
        stribeck_velocity=0.1,  # 스트리벡 천이 속도 (rad/s)
        gear_efficiency=0.8,  # 기어 효율 80%
    )

    # ──────────────────────────────────────────────────────────
    # 3. PID 게인 설정
    # ──────────────────────────────────────────────────────────
    pid_gains = PIDGains(
        kp=100.0,  # 비례 게인 — 위치 오차에 대한 강성(stiffness)
        ki=10.0,  # 적분 게인 — 정상상태 오차 제거
        kd=15.0,  # 미분 게인 — 감쇠(damping)
        i_max=50.0,  # 적분 와인드업 한계
        d_filter=0.1,  # 미분 LPF 계수
    )

    # ──────────────────────────────────────────────────────────
    # 4. 엔진 생성
    # ──────────────────────────────────────────────────────────
    engine = MotorPhysicsEngine(
        motor_params=motor_params,
        friction_params=friction_params,
        pid_gains=pid_gains,
        joint_name="test_joint_1",
    )

    # ──────────────────────────────────────────────────────────
    # 5. 가상 시뮬레이션 루프 (MuJoCo 없이)
    # ──────────────────────────────────────────────────────────
    dt = 0.002  # 2ms 타임스텝 (500Hz)
    duration = 3.0  # 3초 시뮬레이션
    num_steps = int(duration / dt)

    # 사인파 목표 궤적 생성 (0.5 Hz, ±1.0 rad)
    times = np.arange(0, duration, dt)
    trajectory = 1.0 * np.sin(2.0 * np.pi * 0.5 * times)

    # 간단한 1-DOF 시뮬레이션 (오일러 적분)
    # MuJoCo 없이 물리 파이프라인만 검증
    q_act = 0.0  # 초기 위치
    v_act = 0.0  # 초기 속도
    inertia = 0.1  # 관성 모멘트 (kg·m²)

    print("=" * 60)
    print("  모터 검증 시뮬레이션 시작 (MuJoCo 없이)")
    print(f"  dt={dt}s, duration={duration}s, steps={num_steps}")
    print("=" * 60)

    for i in range(num_steps):
        t = times[i]
        q_ref = trajectory[i]

        # 물리 파이프라인 실행 (Step 1~5)
        state = engine.compute_torque(
            time=t,
            q_ref=q_ref,
            q_act=q_act,
            v_act=v_act,
            dt=dt,
        )

        # 간단한 오일러 적분 (MuJoCo 대체)
        # a = T / J (뉴턴 제2법칙의 회전 버전)
        acceleration = state.tau_final / inertia
        v_act += acceleration * dt
        q_act += v_act * dt

    # ──────────────────────────────────────────────────────────
    # 6. 결과 출력
    # ──────────────────────────────────────────────────────────
    engine.print_report()

    csv_path = engine.export_log("motor_validation_log.csv")
    print(f"  로그 저장: {csv_path}")

    # ──────────────────────────────────────────────────────────
    # 7. 그래프 출력 (matplotlib)
    # ──────────────────────────────────────────────────────────
    try:
        import matplotlib.pyplot as plt

        data = engine.logger.to_numpy()

        fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
        fig.suptitle("Motor Validation Pipeline — Test Joint", fontsize=14)

        # (1) 위치 추종
        axes[0].plot(
            data["time"], data["q_ref"], "b--", label="q_ref (Target)", linewidth=1
        )
        axes[0].plot(
            data["time"], data["q_act"], "r-", label="q_act (Actual)", linewidth=1
        )
        axes[0].set_ylabel("Position (rad)")
        axes[0].legend(loc="upper right")
        axes[0].grid(True, alpha=0.3)

        # (2) 토크 파이프라인
        axes[1].plot(
            data["time"],
            data["tau_command"],
            "gray",
            label="T_cmd (PID)",
            linewidth=0.8,
        )
        axes[1].plot(
            data["time"], data["tau_limited"], "orange", label="T_limited", linewidth=1
        )
        axes[1].plot(
            data["time"], data["tau_final"], "r-", label="T_final", linewidth=1.2
        )
        axes[1].axhline(
            y=motor_params.rated_torque, color="green", linestyle=":", label="Rated"
        )
        axes[1].axhline(y=-motor_params.rated_torque, color="green", linestyle=":")
        axes[1].set_ylabel("Torque (Nm)")
        axes[1].legend(loc="upper right", fontsize=8)
        axes[1].grid(True, alpha=0.3)

        # (3) 마찰 토크
        axes[2].plot(
            data["time"],
            data["tau_friction"],
            "purple",
            label="T_friction",
            linewidth=1,
        )
        axes[2].set_ylabel("Friction (Nm)")
        axes[2].legend(loc="upper right")
        axes[2].grid(True, alpha=0.3)

        # (4) T-N 커브 산점도
        axes[3].scatter(
            np.abs(data["v_act"]),
            np.abs(data["tau_final"]),
            c=data["time"],
            cmap="viridis",
            s=1,
            alpha=0.5,
        )
        # T-N 커브 경계선
        v_range = np.linspace(0, motor_params.max_velocity, 100)
        t_curve = motor_params.rated_torque * (1 - v_range / motor_params.max_velocity)
        axes[3].plot(v_range, t_curve, "r--", label="T-N Curve Limit", linewidth=2)
        axes[3].set_xlabel("Speed (rad/s)")
        axes[3].set_ylabel("|Torque| (Nm)")
        axes[3].legend(loc="upper right")
        axes[3].grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig("motor_validation_plot.png", dpi=150)
        print(f"  그래프 저장: motor_validation_plot.png")
        plt.show()

    except ImportError:
        print("  [INFO] matplotlib가 없어 그래프를 건너뜁니다.")


def run_validation_with_mujoco() -> None:
    """
    MuJoCo 시뮬레이션 루프와 통합하는 예제.

    실제 MuJoCo 모델을 로드하고 motor_validation 파이프라인을
    mj_step 루프 내에서 실행합니다.
    """
    try:
        import mujoco
    except ImportError:
        print("[SKIP] mujoco 패키지가 설치되어 있지 않습니다.")
        print("       pip install mujoco 를 실행하세요.")
        return

    # ── 모터 및 마찰 파라미터 설정 (위와 동일) ──
    motor_params = MotorParams(
        rated_torque=1.5,
        peak_torque=4.5,
        max_velocity=5.2,
        gear_ratio=100.0,
    )
    friction_params = FrictionParams(
        coulomb_friction=0.1,
        viscous_friction=0.01,
        stribeck_friction=0.05,
        breakaway_friction=0.15,
        stribeck_velocity=0.1,
        gear_efficiency=0.8,
    )
    pid_gains = PIDGains(kp=100.0, ki=10.0, kd=15.0)

    engine = MotorPhysicsEngine(
        motor_params=motor_params,
        friction_params=friction_params,
        pid_gains=pid_gains,
        joint_name="mujoco_joint_1",
    )

    # ── MuJoCo 모델 로드 ──
    # 실제 사용 시 아래 경로를 실제 MJCF 파일로 교체하세요
    # model = mujoco.MjModel.from_xml_path("my_robot.xml")

    # 테스트용 최소 MJCF 모델 (인라인)
    MINIMAL_MJCF = """
    <mujoco>
      <option timestep="0.002"/>
      <worldbody>
        <body name="link1">
          <joint name="joint1" type="hinge" axis="0 0 1" armature="0.005"/>
          <geom type="capsule" size="0.02 0.1" mass="0.5"/>
        </body>
      </worldbody>
      <actuator>
        <motor name="motor1" joint="joint1" gear="1" ctrllimited="true"
               ctrlrange="-5 5"/>
      </actuator>
    </mujoco>
    """

    model = mujoco.MjModel.from_xml_string(MINIMAL_MJCF)
    data = mujoco.MjData(model)

    dt = model.opt.timestep
    duration = 3.0
    num_steps = int(duration / dt)

    # 목표 궤적
    times = np.arange(0, duration, dt)
    trajectory = 1.0 * np.sin(2.0 * np.pi * 0.5 * times)

    # 관절/액추에이터 인덱스
    joint_id = 0  # data.qpos[joint_id]
    actuator_id = 0  # data.ctrl[actuator_id]

    print("=" * 60)
    print("  MuJoCo 모터 검증 시뮬레이션 시작")
    print(f"  dt={dt}s, duration={duration}s, steps={num_steps}")
    print("=" * 60)

    # ── 시뮬레이션 루프 ──
    for i in range(num_steps):
        t = times[i]
        q_ref = trajectory[i]

        # 현재 상태를 MuJoCo에서 읽기
        q_act = data.qpos[joint_id]
        v_act = data.qvel[joint_id]

        # ┌──────────────────────────────────────────────────┐
        # │  핵심: motor_validation 파이프라인 실행          │
        # │  Step 1~5를 실행하고 Step 6에서 data.ctrl에 적용 │
        # └──────────────────────────────────────────────────┘
        state = engine.step(
            time=t,
            q_ref=q_ref,
            q_act=q_act,
            v_act=v_act,
            dt=dt,
            mj_data=data,
            actuator_id=actuator_id,
        )

        # MuJoCo 물리 엔진 한 스텝 전진
        mujoco.mj_step(model, data)

    # ── 결과 출력 ──
    engine.print_report()
    engine.export_log("motor_validation_mujoco_log.csv")


# ──────────────────────────────────────────────────────────────
# 메인 실행
# ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("\n[1/2] MuJoCo 없이 파이프라인 검증...")
    run_validation_without_mujoco()

    print("\n[2/2] MuJoCo 통합 검증...")
    run_validation_with_mujoco()
