# Motor Parameter Validation Guide

## Overview

The motor validation system provides automatic verification that your default motor parameters work correctly with your robot model and trajectory.

## Quick Start

After exporting your robot model:

```bash
cd <exported_package_name>
./run_torque_replay.sh
# Select option 0 for Quick Validation
```

The validation runs a 5-second simulation and checks:
- ✅ **Tracking accuracy**: Can motors follow the trajectory?
- ✅ **Force saturation**: Are motors reaching their limits?
- ✅ **Stability**: Is the robot oscillating or stable?

## Validation Modes

### Mode 0: Quick Validation (NEW)
**Purpose**: Verify default parameters work before tuning

**When to use**:
- After exporting a new robot
- After changing trajectory significantly
- Before detailed per-joint tuning

**Output**:
```
MOTOR PARAMETER VALIDATION
===========================================================
Model: robot_model.xml
Testing default control and motor parameters...

Running simulation...

Test Duration: 2.50s (500 steps)

Results:
  Tracking Error:
    Average: 0.0234 rad (1.34°)
    Maximum: 0.0891 rad (5.11°)
  Force Usage:
    Average: 12.34 Nm
    Peak: 45.67 Nm
    Saturation: 8.2%
  Stability Score: 0.85/1.00

===========================================================
✅ VALIDATION PASSED
Default parameters work well for this trajectory!
You can now adjust individual motor specs per joint.
===========================================================
```

### Mode 1: Joint Torque Visualization
**Purpose**: See theoretical torque requirements (inverse dynamics)

**When to use**: 
- Understanding torque profiles
- Identifying peak loads

### Mode 2: Motor Sizing Validation
**Purpose**: Interactive parameter tuning with real-time feedback

**When to use**:
- After validation passes (Mode 0)
- Fine-tuning per-joint motor specifications
- Testing different motor combinations

**Features**:
- Global control parameters: `kp` (position gain), `kv` (damping)
- Per-joint motor specs: `gear`, `forcelim`, `velocity`, `armature`, `friction`, `damping`
- 20 joints per page with pagination
- Save/load parameter sets

### Mode 3: Fingertip Sensor Forces
**Purpose**: Contact force visualization

**When to use**: Grasping or manipulation tasks

## Default Parameters

### Control Parameters (Global, Software)
- `kp = 200` - Position gain (reduced for stability)
- `kv = 40` - Velocity damping (20% damping ratio)

### Motor Parameters (Per-Joint, Hardware)
- `gear = 1.0` - Gear ratio
- `forcelim = 80` Nm - Maximum motor force
- `ctrlrange_max = 10` rad/s - Maximum velocity
- `armature = 0.001` - Motor inertia
- `frictionloss = 0.1` - Friction coefficient
- `damping = 0.5` - Joint passive damping

## Validation Thresholds

The validation script checks these criteria:

| Metric | Threshold | Reason |
|--------|-----------|--------|
| Average Tracking Error | < 0.15 rad (~8.6°) | Acceptable accuracy |
| Force Saturation | < 20% | Motors not overloaded |
| Stability Score | > 0.7 | Minimal oscillation |

## Troubleshooting

### ❌ High Tracking Error
**Problem**: Robot can't follow trajectory accurately

**Solutions**:
1. Increase `kp` (position gain) - makes system more responsive
2. Check if trajectory is too fast for motors
3. Consider increasing `forcelim` if forces are saturated

**Example**:
```
Issues detected:
  - High tracking error (12.3° > 8.6°)

Recommendations:
  - Increase kp (position gain) for better tracking
  - Check if trajectory is too fast for current motors
```

### ❌ High Force Saturation
**Problem**: Motors hitting force limits too often

**Solutions**:
1. Increase `forcelim` (motor force limit) - use stronger motors
2. Reduce `kp` - demand less force
3. Slow down trajectory

**Example**:
```
Issues detected:
  - High saturation rate (35.2% > 20.0%)

Recommendations:
  - Increase forcelim (motor force limit)
  - Or reduce kp to demand less force
```

### ❌ Poor Stability
**Problem**: Robot oscillating or vibrating

**Solutions**:
1. Increase `kv` (velocity damping) - add damping
2. Increase joint `damping` - add passive damping
3. Reduce `kp` - less aggressive control

**Example**:
```
Issues detected:
  - Poor stability (0.45 < 0.7)

Recommendations:
  - Increase kv (damping) for stability
  - Add joint damping if oscillating
```

## Workflow

### Recommended Workflow

1. **Export robot model** from Robot Link Forge
2. **Run validation** (Mode 0)
   ```bash
   ./run_torque_replay.sh
   # Select 0
   ```
3. **If validation passes ✅**:
   - Proceed to Mode 2 for per-joint tuning
   - Adjust motor specs for specific joints
   - Test different motor combinations
   
4. **If validation fails ❌**:
   - Follow recommendations
   - Adjust global parameters in MJCF exporter
   - Re-export and validate again

### Parameter Tuning Workflow (Mode 2)

1. Start with validated defaults
2. Select joint(s) to modify
3. Adjust motor parameters:
   - Higher `gear`: Increase torque, reduce speed
   - Higher `forcelim`: Allow more force
   - Lower `ctrlrange_max`: Limit velocity
4. Observe real-time tracking error
5. Save parameter set when satisfied
6. Run post-analysis:
   ```bash
   python3 analyze_motor_validation.py motor_validation_log.csv
   ```

## Understanding Damping Ratio

The default parameters provide **20% damping** (near-critically damped):

- **< 10% damping**: Oscillatory, unstable
- **10-30% damping**: Good balance (recommended)
- **> 50% damping**: Sluggish response

Formula: `damping_ratio = kv / (2 * sqrt(kp))`

For `kp=200, kv=40`: `damping_ratio = 40 / (2 * sqrt(200)) ≈ 0.20` (20%)

## Files Generated

After export, your package contains:

```
<package_name>/
├── robot_model.xml              # MuJoCo MJCF
├── recordings.json              # Motion data
├── run_torque_replay.sh         # Interactive launcher
├── validate_motor_params.py     # Quick validation (NEW)
├── replay_with_torque.py        # Mode 1: Inverse dynamics
├── replay_motor_validation.py   # Mode 2: Parameter tuning
├── analyze_motor_validation.py  # Post-processing analysis
└── venv/                        # Auto-created virtual env
```

## Advanced Usage

### Running Validation Directly

```bash
python3 validate_motor_params.py robot_model.xml
```

Exit codes:
- `0`: Validation passed
- `1`: Validation failed

### Scripting

```bash
#!/bin/bash
if python3 validate_motor_params.py robot_model.xml; then
    echo "Parameters OK, proceeding with testing..."
    # Run your tests
else
    echo "Parameters need adjustment"
    exit 1
fi
```

### Custom Thresholds

Edit `validate_motor_params.py` to adjust thresholds:

```python
# At top of file
MAX_ACCEPTABLE_ERROR = 0.10  # More strict (0.15 default)
MAX_SATURATION_PCT = 15.0    # More strict (20.0 default)
MIN_STABILITY_SCORE = 0.8    # More strict (0.7 default)
```

## Technical Details

### Validation Algorithm

1. Load MuJoCo model and recording
2. Build trajectory (5 seconds max)
3. Run simulation with default parameters
4. Measure every step:
   - Tracking error: `|target_position - actual_position|`
   - Force: `actuator_force` vs `forcelim`
   - Saturation: Count when force > 95% of limit
5. Calculate stability: `1.0 / (1.0 + std(errors) * 10)`
6. Compare metrics against thresholds
7. Generate pass/fail verdict with recommendations

### Parameter Storage

Parameters are saved in JSON format:

```json
{
  "control": {
    "kp": 200,
    "kv": 40
  },
  "motor_global": {
    "gear": 1.0,
    "forcelim": 80,
    "ctrlrange_max": 10,
    "armature": 0.001,
    "frictionloss": 0.1,
    "damping": 0.5
  },
  "per_joint": {
    "finger_joint_1": {
      "gear": 2.0,
      "forcelim": 100
    }
  }
}
```

## FAQ

**Q: How long does validation take?**  
A: ~5 seconds (500 simulation steps)

**Q: Can I skip validation?**  
A: Yes, it's optional. But recommended for new robots.

**Q: What if validation fails repeatedly?**  
A: Your trajectory might be too aggressive for the default motor specs. Consider:
- Slowing down the motion
- Using the trajectory at reduced speed in Mode 2
- Adjusting MJCF exporter defaults in the code

**Q: Can I validate without GUI?**  
A: Yes, run `python3 validate_motor_params.py` directly (headless simulation)

**Q: Where are default parameters defined?**  
A: In `src/backend/exporters/mjcf_exporter.py` and `motion_exporter.py`

**Q: Can I change defaults permanently?**  
A: Yes, edit the exporter files before exporting your robot

## Summary

The validation system ensures your default parameters work before you spend time on detailed tuning. It's fast (5 seconds), automatic, and provides actionable feedback. Always run Mode 0 validation first, then proceed to Mode 2 for per-joint optimization only if needed.

**Workflow**: Export → Validate (Mode 0) → Tune (Mode 2) → Deploy
