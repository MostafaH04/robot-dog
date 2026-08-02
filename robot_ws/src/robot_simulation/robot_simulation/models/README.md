# MuJoCo teaching models

The simulator ships two selectable models. Both implement the same 12 named
joints, actuator names, IMU site, foot geom/site names, fixed 2 ms step, solver
settings, and safe-stance reset contract.

```python
from robot_simulation.mujoco_core import MujocoSimulator

baseline = MujocoSimulator()  # model_variant="primitive"
enhanced = MujocoSimulator(model_variant="enhanced")
```

`minimal_quadruped.xml` remains the Phase 2 known-stable teaching baseline. It
uses a 6 kg box torso, capsule legs, spherical feet, and deliberately approximate
mass and geometry.

`enhanced_quadruped.xml` is the optional Phase 3 model. It is higher fidelity in
specific, bounded ways, but it is not a validated physical digital twin.

## Enhanced-model provenance

All source files below are checked into this repository:

| Model value | Repository source | Treatment |
| --- | --- | --- |
| Base and moving-link mass, center of mass, and inertia tensors | `robot_ws/src/robot_simulation/urdf/robot_core.xacro` link `<inertial>` elements | Copied to explicit MJCF `<inertial>` elements. The source model's right-positive lateral coordinate is reflected to the core's documented y-left convention; products involving y are sign-adjusted. Values are rounded to the shown MJCF precision. |
| Hip and serial leg joint anchors | The same Xacro joint `<origin>` elements | Copied, with the lateral coordinate reflected. Front/rear and left/right placement now follows the export instead of the Phase 2 approximate anchors. |
| Base and link primitive envelopes | Bounds of the checked-in `urdf/meshes/*.stl` files at their declared `0.001` scale | Dense meshes are replaced with boxes/capsules. Base half-extents use the `base_link.stl` envelope. Hip boxes use mount envelopes. Each leg capsule centerline uses approximately twice the exported center-of-mass offset, while its radius uses half the narrow mesh dimension. This is a documented collision approximation, not recovered CAD dynamics. |
| Joint names and leg ordering | Active Xacro plus the established `/cmd_jnts` and Phase 2 core contract | Preserved exactly. |

The old export has useful geometry and inertial facts, but it does not establish
validated hardware properties. In particular, its ±π joint limits, 10,000 N·m
effort limits, and 100 rad/s velocity limits are generic export values and are
not treated as physical specifications.

## Known approximations and tuning parameters

- The enhanced model remains the active open-chain, three-joint-per-leg teaching
  abstraction. It does not reconstruct the physical parallel/closed-chain links
  or add equality constraints.
- Mirrored joint-axis signs stay on the Phase 2 canonical command convention.
  The older export's per-leg sign choices are coordinate/export conventions,
  not evidence of measured hardware-positive directions.
- Primitive collision shapes approximate the checked-in mesh envelopes. Dense
  STL collision, mesh decimation, and CAD recovery are deliberately deferred.
- The Phase 2 conservative command ranges are retained: hip `[-0.6, 0.6]`, upper
  leg `[-1.2, 1.2]`, and lower leg `[-2.0, 0.3]` radians. These are safe teaching
  bounds, not measured hardware limits.
- Position gains (`kp=55`, `kv=4`), force range (±35), joint damping (`0.8`),
  armature (`0.01`), friction (`0.9 0.02 0.001`), and contact solver parameters
  are inherited simulation tuning. The repository contains no motor curve,
  gearbox, backlash, compliance, foot-material, or contact-identification data
  that would justify a hardware claim.
- The foot capsule is both the collision object and the geom accumulated by the
  existing mock contact/normal-force/wrench interface. The transparent site at
  its endpoint is the wrench reference. These signals remain ideal MuJoCo
  contact outputs, not load-cell or pressure-sensor models.
- The IMU remains noise-free and colocated with the base origin. There is no
  evidence-backed sensor pose, bias, noise, saturation, delay, or calibration
  model in the repository.

## Non-goals

This layer does not add ROS or Gymnasium adapters, an estimator, locomotion
policy training, camera/vision support, detailed CAD recovery, actuator system
identification, real-hardware validation, or cross-platform bitwise guarantees.
Use the primitive variant when following the smallest stable teaching fixture;
use enhanced when an exercise benefits from the repository-exported mass and
geometry structure.
