#!/usr/bin/env python3
"""
Physics Diagnosis Tool
Checks for common issues in robot model that cause instability
"""
import mujoco
import numpy as np
import sys
import os

def diagnose_mjcf(model_path):
    """Diagnose common physics issues in MJCF model"""
    
    if not os.path.exists(model_path):
        print(f"ERROR: Model file not found: {model_path}")
        return
    
    print("="*70)
    print("MUJOCO PHYSICS DIAGNOSIS")
    print("="*70)
    print(f"Model: {model_path}\n")
    
    # Load model
    try:
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"ERROR loading model: {e}")
        return
    
    # 1. Timestep check
    print("1. TIMESTEP")
    print("-" * 70)
    timestep = model.opt.timestep
    print(f"   Timestep: {timestep*1000:.3f} ms")
    if timestep > 0.002:
        print(f"   [WARNING] Timestep too large! Recommended: < 2ms")
        print(f"   Large timestep causes instability and explosions")
    elif timestep < 0.0001:
        print(f"   [WARNING] Timestep very small, simulation will be slow")
    else:
        print(f"   [OK] Timestep reasonable")
    print()
    
    # 2. Mass distribution
    print("2. BODY MASSES")
    print("-" * 70)
    total_mass = 0
    suspicious_bodies = []
    
    for i in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        mass = model.body_mass[i]
        total_mass += mass
        
        if mass > 10.0:
            suspicious_bodies.append((name, mass, "very heavy"))
        elif mass > 0 and mass < 0.001:
            suspicious_bodies.append((name, mass, "very light"))
    
    print(f"   Total robot mass: {total_mass:.3f} kg")
    print(f"   Number of bodies: {model.nbody}")
    print(f"   Average body mass: {total_mass/model.nbody:.3f} kg")
    
    if total_mass > 50:
        print(f"   [WARNING] Robot is very heavy ({total_mass:.1f} kg)")
        print(f"   Heavy robots need very high motor torques")
    elif total_mass < 0.1:
        print(f"   [WARNING] Robot is very light ({total_mass:.3f} kg)")
        print(f"   Light robots may be unstable")
    else:
        print(f"   [OK] Total mass reasonable")
    
    if suspicious_bodies:
        print(f"\n   Suspicious bodies:")
        for name, mass, reason in suspicious_bodies[:5]:
            print(f"     - {name}: {mass:.6f} kg ({reason})")
    print()
    
    # 3. Inertia check
    print("3. INERTIA VALUES")
    print("-" * 70)
    zero_inertia_count = 0
    extreme_inertia = []
    
    for i in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        # Inertia is in body_inertia (3 values: ixx, iyy, izz)
        inertia = model.body_inertia[i]
        
        if np.all(inertia == 0):
            zero_inertia_count += 1
        elif np.any(inertia > 1.0):
            extreme_inertia.append((name, inertia))
        elif np.any(inertia < 1e-8) and np.any(inertia > 0):
            extreme_inertia.append((name, inertia))
    
    print(f"   Bodies with zero inertia: {zero_inertia_count}")
    if extreme_inertia:
        print(f"   Bodies with extreme inertia:")
        for name, inertia in extreme_inertia[:5]:
            print(f"     - {name}: [{inertia[0]:.2e}, {inertia[1]:.2e}, {inertia[2]:.2e}]")
    
    if zero_inertia_count > model.nbody * 0.5:
        print(f"   [WARNING] Many bodies have zero inertia!")
        print(f"   This means STL inertia calculation may have failed")
    print()
    
    # 4. Actuator check
    print("4. ACTUATORS")
    print("-" * 70)
    print(f"   Number of actuators: {model.nu}")
    
    weak_actuators = []
    strong_actuators = []
    
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        kp = model.actuator_gainprm[i, 0]
        kv = model.actuator_biasprm[i, 1]
        gear = model.actuator_gear[i, 0]
        forcerange = model.actuator_forcerange[i]
        
        max_force = forcerange[1]
        max_torque = abs(gear * max_force)
        
        if max_torque < 10:
            weak_actuators.append((name, max_torque, gear, kp))
        elif max_torque > 100000:
            strong_actuators.append((name, max_torque, gear, kp))
    
    if weak_actuators:
        print(f"\n   [WARNING] Weak actuators (max torque < 10 Nm):")
        for name, torque, gear, kp in weak_actuators[:5]:
            print(f"     - {name}: {torque:.1f} Nm (gear={gear:.0f}, kp={kp:.0f})")
        print(f"   Weak actuators cannot move heavy links!")
    
    if strong_actuators:
        print(f"\n   [WARNING] Very strong actuators (max torque > 100k Nm):")
        for name, torque, gear, kp in strong_actuators[:5]:
            print(f"     - {name}: {torque:.0f} Nm (gear={gear:.0f}, kp={kp:.0f})")
        print(f"   Extremely high torque can cause instability!")
    
    if not weak_actuators and not strong_actuators:
        print(f"   [OK] Actuator torques seem reasonable")
    print()
    
    # 5. Control gains check
    print("5. CONTROL GAINS")
    print("-" * 70)
    high_kp = []
    low_damping = []
    
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        kp = model.actuator_gainprm[i, 0]
        kv = -model.actuator_biasprm[i, 1]  # Negative in MuJoCo
        
        if kp > 5000:
            high_kp.append((name, kp, kv))
        
        if kp > 0 and kv / kp < 0.05:  # Less than 5% damping
            low_damping.append((name, kp, kv, kv/kp*100))
    
    if high_kp:
        print(f"   [WARNING] Very high kp gains (> 5000):")
        for name, kp, kv in high_kp[:5]:
            print(f"     - {name}: kp={kp:.0f}, kv={kv:.0f}")
        print(f"   High kp can cause oscillations and instability!")
    
    if low_damping:
        print(f"\n   [WARNING] Low damping (< 5%):")
        for name, kp, kv, ratio in low_damping[:5]:
            print(f"     - {name}: kp={kp:.0f}, kv={kv:.0f} ({ratio:.1f}%)")
        print(f"   Low damping causes oscillations!")
    
    if not high_kp and not low_damping:
        print(f"   [OK] Control gains seem balanced")
    print()
    
    # 6. Joint check
    print("6. JOINTS")
    print("-" * 70)
    print(f"   Number of joints: {model.njnt}")
    
    unlimited_joints = []
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        jnt_range = model.jnt_range[i]
        
        # Check if range is effectively unlimited
        if jnt_range[1] - jnt_range[0] > 6.0:  # > ~360 degrees
            unlimited_joints.append(name)
    
    if unlimited_joints:
        print(f"   [INFO] Joints with wide range (> 360 deg):")
        for name in unlimited_joints[:5]:
            print(f"     - {name}")
    print()
    
    # 7. Summary
    print("="*70)
    print("DIAGNOSIS SUMMARY")
    print("="*70)
    
    issues = []
    
    if timestep > 0.002:
        issues.append("Timestep too large - reduce to < 2ms")
    
    if total_mass > 50:
        issues.append(f"Robot very heavy ({total_mass:.1f} kg) - need high motor torque")
    elif total_mass < 0.1:
        issues.append(f"Robot very light ({total_mass:.3f} kg) - may be unstable")
    
    if zero_inertia_count > model.nbody * 0.5:
        issues.append("Many zero inertias - STL calculation failed?")
    
    if weak_actuators:
        issues.append(f"{len(weak_actuators)} weak actuators - cannot move heavy links")
    
    if strong_actuators:
        issues.append(f"{len(strong_actuators)} extremely strong actuators - may cause instability")
    
    if high_kp:
        issues.append(f"{len(high_kp)} actuators with very high kp - causes oscillations")
    
    if low_damping:
        issues.append(f"{len(low_damping)} actuators with low damping - causes oscillations")
    
    if not issues:
        print("✓ No major issues detected!")
        print("\nIf robot still unstable, check:")
        print("  - Trajectory is not too fast/aggressive")
        print("  - Joint limits are reasonable")
        print("  - No self-collisions in model")
    else:
        print("Issues found:")
        for i, issue in enumerate(issues, 1):
            print(f"  {i}. {issue}")
    
    print()
    
    # 8. Recommendations
    print("="*70)
    print("RECOMMENDATIONS")
    print("="*70)
    
    if total_mass > 10:
        recommended_gear = int(total_mass * 50)
        print(f"For robot mass {total_mass:.1f} kg:")
        print(f"  - Recommended gear ratio: {recommended_gear}")
        print(f"  - Recommended forcelim: {recommended_gear * 5} Nm")
    
    print(f"\nFor stable control:")
    print(f"  - kp: 500-1000 (not > 2000)")
    print(f"  - kv: 10% of kp (damping ratio)")
    print(f"  - gear: {int(total_mass * 50)}-{int(total_mass * 100)}")
    print(f"  - timestep: 0.001-0.002 (1-2 ms)")
    
    print("\n" + "="*70)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python diagnose_physics.py <robot.xml>")
        print("\nSearching for robot.xml in common locations...")
        
        # Search common locations
        search_paths = [
            "robot.xml",
            "src/backend/exported_files/robot.xml",
            "exported_files/robot.xml"
        ]
        
        found = None
        for path in search_paths:
            if os.path.exists(path):
                found = path
                break
        
        if found:
            print(f"Found: {found}\n")
            diagnose_mjcf(found)
        else:
            print("No robot.xml found. Please specify path.")
            sys.exit(1)
    else:
        diagnose_mjcf(sys.argv[1])
