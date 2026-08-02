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
| Closed-chain topology and lengths | `robot_ws/src/robot_controller/robot_controller/leg_kin.py` (`a_len` through `e_len`) and its predecessor `main.py` at Git commit `0d2d729` | Each leg has two paths from a common post-hip-roll anchor to one closure point: commanded `a=0.1059` m then `c=0.047434` m, and passive `b=0.0245` m then `e=0.11058` m. A site-to-site MuJoCo `connect` equality closes the paths. The remaining `d=0.063725` m is the documented distal foot extension. |
| Removed linkage bodies and open-tree anchors | Historical `robot_ws/src/robot_description/urdf/assem.xacro` at commit `1167ae8`; current `Mini_Link*`, `Right_Link*`, `Component2*`, and `Servo_Horn*` meshes; duplicated `robot_trans.trans` files | These prove that auxiliary mechanism bodies existed and preserve exported masses and partial tree anchors. They do not declare closing joints or usable closure transforms, so they are supporting evidence rather than a source for invented CAD endpoint poses. |
| Base and link primitive envelopes | Bounds of the checked-in `urdf/meshes/*.stl` files at their declared `0.001` scale | Dense meshes are replaced with boxes/capsules. Base half-extents use the `base_link.stl` envelope. Hip boxes use mount envelopes. Each leg capsule centerline uses approximately twice the exported center-of-mass offset, while its radius uses half the narrow mesh dimension. This is a documented collision approximation, not recovered CAD dynamics. |
| Joint names and leg ordering | Active Xacro plus the established `/cmd_jnts` and Phase 2 core contract | Preserved exactly. |

The old export has useful geometry and inertial facts, but it does not establish
validated hardware properties. In particular, its ±π joint limits, 10,000 N·m
effort limits, and 100 rad/s velocity limits are generic export values and are
not treated as physical specifications.

## Closed-loop representation

The current `robot_core.xacro` copies under `robot_desc/urdf` and
`robot_simulation/urdf` are intentionally simplified serial descriptions; their
only difference is mesh URI spelling. The original repository README (commit
`1a43c14`) explains that the physical leg is closed-chain, that the detailed
Fusion-to-URDF export had to remain a tree, and that the correct closing relative
transforms were never recovered from Fusion. The historical export therefore
contains 29 links and 28 joints—still a tree—and cannot by itself establish a
faithful CAD closure.

The checked-in kinematics solver is the strongest complete topology source. In
the enhanced MJCF, the existing named upper/lower joints remain the `a-c` path,
preserving the Phase 2 serial command and joint-state convention. Two unactuated
hinges create the `b-e` path. A `connect` equality per leg makes both endpoints
coincident. The magenta, non-colliding capsules make that passive path visible in
the deterministic review animation without changing foot contact semantics.

This is a well-founded ideal planar closure, not a recovered physical linkage.
In particular, the solver co-locates the two proximal planar pivots while the
historical CAD tree shows offset servo axes, and the repository supplies no
closing transforms that reconcile those descriptions. Small lateral offsets in
the export are flattened at the closure site to match the solver's planar model.
The passive-link masses use the historical Mini/Right-link mass magnitudes, but
their capsule inertia tensors are geometric approximations because the missing
closed poses make the exported tensors inapplicable. The passive links do not
collide; primary leg and foot geoms retain collision/contact responsibility.

## Known approximations and tuning parameters

- The public upper/lower positions remain simplified serial coordinates. They
  are not renamed or reinterpreted as the two physical servo-shaft angles. A
  requested serial pose outside the ideal linkage workspace is resolved by the
  constraint and tuned position actuators rather than representing real motor
  control.
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
