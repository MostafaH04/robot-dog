# Minimal MuJoCo teaching model

`minimal_quadruped.xml` is the Phase 2 simulator model. It is deliberately a
small deterministic test fixture, not a digital twin:

- one 6 kg box torso, primitive capsule legs, and spherical feet;
- three hinge joints per leg using the baseline `/cmd_jnts` names and leg order;
- position actuators with bounded force and target ranges;
- a 2 ms fixed timestep, explicit Newton solver settings, and one plane;
- an IMU site plus named foot geoms/sites used by the Python sensor interface;
- a conservative `safe_stance` keyframe used by every reset.

Masses and dimensions are approximate. The model omits the physical closed-chain
mechanism, CAD meshes, actuator dynamics, backlash, compliance, sensor
noise/bias/saturation, cabling, and hardware validation. Those belong to later
model-fidelity work and should not be inferred from this file.
