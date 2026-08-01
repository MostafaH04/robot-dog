# Physical Robot Revival Requirements

**Status:** Planning draft; no hardware is authorized to move from this document alone

**Scope:** Requirements and staged evidence for inspecting, restoring, and eventually operating the physical quadruped

**Repository review point:** `origin/main` at `1a43c14` plus historical refs available on 2026-08-01

## 1. Purpose

This document defines the evidence and decisions needed to revisit the physical Robot Dog without bypassing the simulation-first direction of the project. It is deliberately conservative: the repository records that a physical robot existed and walked, but it does not contain a current hardware bill of materials, wiring diagram, validated physical limits, production firmware, or a safe bring-up procedure.

The requirements below therefore distinguish what the repository confirms from what is proposed. A proposal is not a recovered hardware specification. Unknown values must be measured or decided before the affected bring-up gate can pass.

This document does not:

- authorize energizing actuators;
- claim that the current URDF is a mechanically or dynamically validated digital twin;
- select final actuators, batteries, compute, buses, sensors, or control rates;
- replace an electrical design review, mechanical inspection, or task-specific risk assessment; or
- require hardware-specific logic to leak into locomotion and estimation code that should remain reusable in simulation.

## 2. Requirement language and evidence classes

`SHALL` denotes a requirement for the revival effort. `SHOULD` denotes a strong recommendation. `MAY` denotes an allowed option.

Every claim or target uses one of these evidence classes:

| Class | Meaning |
| --- | --- |
| **Confirmed repository fact** | Directly supported by the current tree or reachable Git history. It may still need inspection against the surviving robot. |
| **Proposed requirement** | A planning constraint recommended here. It is not an existing physical specification. |
| **Open question** | Missing information that must be recovered, measured, or answered. |
| **Decision required** | A choice with an owner and recorded rationale is needed before the named gate. |

Measured facts about the surviving robot should be added to a dated inventory or calibration artifact with the observer, method, units, uncertainty where relevant, and hardware revision. They should not silently replace proposed values in this document.

## 3. Repository evidence and limits

### 3.1 Confirmed repository facts

The following statements are supported by the repository:

- The stated project mission is a custom quadruped for autonomous indoor navigation, with simulation intended to reduce risk before deployment to the real robot. The README says the physical robot existed and that a custom controller board with an STM32 MCU and IMU was being designed. See the [project README](../README.md).
- The historical account says lower-level joint-control firmware and a Matplotlib kinematic model were compared with the physical assembly, and that the robot took early walking steps. That is evidence of a prior prototype, not evidence that the present hardware is complete or safe.
- The physical leg is described as a closed-chain, three-degree-of-freedom mechanism. The active ROS description is a simplified open-chain representation because URDF is a tree format, and software maps physical leg geometry to simulated joints. See the [leg kinematics mapper](../robot_ws/src/robot_controller/robot_controller/leg_kin.py) and [active robot description](../robot_ws/src/robot_desc/urdf/robot_core.xacro).
- The active description contains four leg-root joints and eight downstream joints: twelve revolute joints total. Its broad `+/- pi` position limits, `10000` effort values, and `100` velocity values are model placeholders and are not validated physical limits.
- The current control path accepts `sensor_msgs/JointState` on `/control_inputs`, maps twelve numeric joint indices through closed-chain geometry, and publishes `sensor_msgs/JointState` on `/cmd_jnts`. The simulator consumes `/cmd_jnts`, publishes `/joint_states`, and broadcasts `world -> base_link`. See the [joint controller](../robot_ws/src/robot_controller/robot_controller/quad_joint_controller.py) and [PyBullet bridge](../robot_ws/src/robot_simulation/robot_simulation/pybullet_sim.py).
- The kinematics code contains dimensions derived from earlier work (`0.038`, `0.1059`, `0.0245`, `0.047434`, `0.063725`, and `0.11058` metres). The code itself includes a warning that something may be wrong. These values are useful recovery leads, not accepted metrology.
- Git history contains a January 2024 PlatformIO proof of concept for a Nucleo-F401RE (`50a869d`). It only toggles an LED. A later commit added an otherwise empty `firmware/` placeholder (`7096082`), and no hardware firmware is present on current `main`.
- Two unmerged historical branches explored IMU work: `origin/simulation/imu` at `19d5f3c` and `origin/adding-imu` at `9a71d81`. The reachable changes add a synthetic PyBullet IMU publisher; they do not establish a physical IMU part, mounting transform, driver, calibration, or validated noise model.
- A separate simulation-baseline branch, reviewed at `b6e3e2e`, establishes a reproducible ROS 2 Jazzy direction and describes the existing simulation as an unvalidated checkpoint. It plans a MuJoCo vertical slice and later model fidelity, sensors, estimation, navigation, and learning work. These changes are not assumed to be merged into `main` by this document.

### 3.2 Facts that are not confirmed

The repository does **not** currently confirm:

- the location, completeness, revision, or present condition of the physical robot;
- actuator manufacturer, model, gearing, feedback type, rated voltage, current, torque, speed, thermal limits, or installed count;
- the actual safe joint range, link clearance, hard stops, transmission ratio, backlash, compliance, or load capacity;
- battery chemistry, cell count, capacity, age, protection, connector, charger, fuse, wiring gauge, grounding, or power-rail topology;
- whether a controller PCB was fabricated, populated, or tested;
- a current companion computer, operating system, real-time controller, motor driver, communications bus, remote control, or e-stop circuit;
- installed IMU, joint encoders, contact sensors, perception sensors, or their calibration;
- physical mass, centre of mass, inertia, foot friction, or contact properties; or
- a validated standing controller, stabilized gait, localization system, navigation stack, or hardware acceptance record.

The CAD-derived inertial values in the Xacro and the historical videos described in the README must not be promoted to current hardware specifications without measurement.

## 4. Intended capabilities and operating envelope

### 4.1 Capability requirements

| ID | Requirement | Evidence class |
| --- | --- | --- |
| CAP-01 | The revived system SHALL allow safe inspection, configuration, calibration, command, disable, and observation of each actuator before coordinated motion is attempted. | Proposed requirement |
| CAP-02 | The system SHALL support four legs with three commanded physical degrees of freedom per leg if the hardware inventory confirms the historical 12-actuator assumption. A mismatch SHALL trigger a model/interface decision before code is adapted. | Proposed requirement based on repository fact |
| CAP-03 | The first locomotion objective SHALL be repeatable stand, sit/lower, and low-speed tethered motion on a flat indoor test surface. | Proposed requirement |
| CAP-04 | The robot SHALL expose enough timestamped joint, body-motion, contact, power, command, and safety state for tests to be reproduced and failures diagnosed. | Proposed requirement |
| CAP-05 | Hardware control SHALL remain behind the same logical command/state boundary used by simulation so controllers and estimators can be exercised without target-specific rewrites. | Proposed requirement |
| CAP-06 | Autonomous indoor navigation remains an intended project capability, but perception, mapping, planning, and untethered autonomous motion SHALL be out of scope until the basic locomotion and safety acceptance gates pass. | Confirmed mission; proposed staging |
| CAP-07 | Dynamic gaits, jumping, stairs, outdoor use, operation near people or animals, payload carrying, and unattended charging SHALL be out of scope unless separately risk-assessed and accepted. | Proposed requirement |

### 4.2 Operating-envelope decisions

Before free-standing tests, the team SHALL record:

- test surface, slope, available clearance, overhead support or tether arrangement, and exclusion zone;
- maximum commanded joint position, velocity, acceleration, effort/current, and temperature for each test stage;
- maximum battery state, test duration, and permitted voltage/current/temperature envelope;
- whether an operator must remain hands-on at the e-stop; and
- fall surfaces, fixture loads, robot lift points, and the method for recovering a disabled or fallen robot.

All numerical values are **decision required**. No values are recoverable with adequate confidence from the current repository.

## 5. Mechanical and actuator requirements

### 5.1 Mechanical recovery

| ID | Requirement | Gate |
| --- | --- | --- |
| MEC-01 | The team SHALL create a photographed, revision-marked inventory of frame parts, links, fasteners, bearings/bushings, shafts, servo horns, feet, wiring restraints, covers, and missing or damaged items. | Before any powered actuator test |
| MEC-02 | Every leg SHALL be moved through its passive range with actuators mechanically unloaded or safely backdrivable. Binding, collisions, loose joints, cracked printed parts, bent shafts, sharp edges, and cable pinch points SHALL be recorded and resolved. | Before any powered actuator test |
| MEC-03 | The closed-chain topology, link dimensions, joint axes, actuator orientation, sign convention, and mechanical reduction SHALL be measured against the surviving assembly and compared with the kinematics mapper and CAD-derived model. | Before coordinated leg motion |
| MEC-04 | Physical hard stops and collision-free soft ranges SHALL be measured per actuator/leg. The placeholder URDF ranges SHALL NOT be used as hardware limits. | Before motor enable |
| MEC-05 | Each load-bearing fastener SHALL have a documented retention method and inspection status. Torque values are **open questions** until the materials, fasteners, and inserts are identified. | Before supported full-body motion |
| MEC-06 | A test fixture SHALL restrain the body and the leg under test without loading unverified parts or preventing access to the e-stop. The fixture load rating is a **decision required** after robot mass is measured. | Before powered leg tests |
| MEC-07 | Feet SHALL be secure and provide a consistent test contact surface. Foot material, friction, wear limits, and replaceability are **open questions**. | Before ground contact tests |

### 5.2 Actuator characterization

The historical use of the word “servo” does not establish the installed actuator type or feedback capabilities. For every installed actuator, ACT-01 SHALL record:

- manufacturer, model, serial/revision if present, supply range, driver, connector, and pinout;
- command mode (for example PWM position, bus position, velocity, current/torque, or another mode);
- feedback available at the actuator and externally: position, velocity, current/effort, voltage, temperature, and faults;
- gearing, direction, neutral/home reference, usable mechanical range, backlash, and observed compliance;
- rated and tested current/torque/speed/temperature limits, with sources and test method; and
- fail behaviour on signal loss, bus loss, brownout, controller reset, and power removal.

Additional requirements:

| ID | Requirement | Evidence class |
| --- | --- | --- |
| ACT-02 | Actuators SHALL first be characterized one at a time with a current-limited supply, no body load, a physical restraint, and command limits narrower than the measured collision-free range. | Proposed requirement |
| ACT-03 | Software position, velocity, acceleration/slew, effort/current, and thermal limits SHALL be derived from measured hardware behaviour and conservative engineering review. | Proposed requirement |
| ACT-04 | The command path SHALL saturate safely and report saturation; it SHALL NOT silently wrap, extrapolate outside calibration, or substitute stale commands. | Proposed requirement |
| ACT-05 | If actuator feedback is absent or cannot independently establish actual position, the team SHALL decide whether external joint encoders are mandatory before load-bearing motion. | Decision required before supported full-body motion |
| ACT-06 | The team SHALL document whether actuator power removal permits a hazardous collapse and shall provide mechanical support or a controlled-lowering strategy appropriate to each test. | Proposed requirement |

## 6. Compute, firmware, and power

### 6.1 Compute architecture

The following split is **proposed**, not recovered from the repository:

- A deterministic low-level controller owns actuator I/O, per-joint limits, command freshness, watchdogs, and immediate fault handling.
- A companion computer runs ROS 2, controller/estimator orchestration, logging, visualization, and later navigation or learning workloads.
- The physical safety chain can remove or inhibit actuator energy without depending on ROS 2, Wi-Fi, the companion computer, or a healthy application process.

Requirements:

| ID | Requirement |
| --- | --- |
| CMP-01 | The MCU/controller, companion computer, operating system, middleware version, real-time needs, boot time, thermal environment, connectors, and mounting SHALL be selected and documented before system integration. |
| CMP-02 | Low-level firmware SHALL start with outputs disabled, validate its configuration, publish a boot/reset reason, and require an explicit enable sequence after every reset. |
| CMP-03 | Firmware and host software SHALL expose semantic version or Git revision, build configuration, hardware revision compatibility, and calibration-set identifier in logs. |
| CMP-04 | A hardware-in-the-loop mode SHALL exercise framing, commands, telemetry, faults, and watchdogs without energizing actuators. |
| CMP-05 | Firmware update and recovery procedures SHALL prevent a partial update from enabling motion and SHALL define how a known-good image is restored. |

The historical Nucleo-F401RE proof of concept is a recovery clue only. Reusing it, selecting another MCU, or using smart actuators with distributed controllers is a **decision required** after actuator and I/O inventory.

### 6.2 Power architecture

| ID | Requirement | Gate |
| --- | --- | --- |
| PWR-01 | A schematic-level power tree SHALL identify energy source, charger, battery protection/BMS, main disconnect, e-stop contactor or inhibit path, fusing, pre-charge/inrush handling if needed, actuator rail, logic rails, converters, grounding, and connectors. | Before connecting a battery |
| PWR-02 | Logic and actuator power SHALL be independently controllable so communication, sensors, and logs can be verified with actuators disabled. | Before integrated bring-up |
| PWR-03 | Every conductor, connector, fuse, switch, converter, and distribution element SHALL be rated from a documented continuous and peak load budget with margin. | Before actuator power |
| PWR-04 | Reverse polarity, short circuit, over-current, under-voltage, over-voltage where relevant, and over-temperature hazards SHALL have documented protection and test evidence. | Before actuator power |
| PWR-05 | Bus voltage and total current SHALL be measured and logged. Per-rail and per-actuator current SHOULD be measured where the architecture permits. | Before full-body motion |
| PWR-06 | Battery chemistry, cell count, age/condition, capacity, charge/discharge limits, storage voltage, approved charger, fire response, transport, and disposal are **open questions**. An unidentified or damaged pack SHALL NOT be used. | Before charging or connecting a battery |
| PWR-07 | Brownout behaviour SHALL be tested with actuator energy inhibited. A brownout or controller reset SHALL leave actuators disabled until explicitly re-enabled. | Before supported full-body motion |

## 7. Sensing and state requirements

### 7.1 Minimum sensing

| ID | Requirement | Evidence class |
| --- | --- | --- |
| SNS-01 | A body-mounted IMU SHALL provide timestamped angular velocity and linear acceleration in a documented ROS frame. Orientation MAY be published only when its estimation method and covariance are defined. | Proposed requirement |
| SNS-02 | Actual physical joint position SHALL be observable for every commanded degree of freedom before load-bearing coordinated motion. Integrated actuator feedback MAY satisfy this after validation; otherwise external encoders are required. | Proposed requirement |
| SNS-03 | Joint-state samples SHALL identify names, units, direction, timestamp, validity, and source. Estimated velocity/effort SHALL be distinguishable from measured values. | Proposed requirement |
| SNS-04 | The system SHALL expose a per-foot contact estimate with confidence or validity before contact-aware estimation/control is accepted. The first standing test MAY use a simpler operator-observed contact state if the controller does not depend on contact feedback. | Proposed requirement |
| SNS-05 | Sensor streams used together SHALL share a time base or have a measured synchronization/transport offset. Timestamping SHOULD occur at acquisition rather than after an unbounded queue. | Proposed requirement |
| SNS-06 | Perception sensors for autonomous navigation are deferred. Their field of view, range, lighting, compute, bandwidth, and mounting requirements SHALL be defined after basic locomotion passes. | Proposed staging |

### 7.2 IMU alternatives and selection

The repository confirms only that an IMU was planned and synthetic IMU work was attempted. The physical part and interface are **open questions**.

The IMU decision SHALL compare:

- gyroscope and accelerometer range, noise, bias stability, output rate, latency, saturation and clipping indicators;
- timestamp/source synchronization, interface bandwidth, driver maturity, temperature behaviour, and calibration support;
- mounting rigidity, vibration isolation trade-offs, axis alignment, location relative to `base_link`, and serviceability; and
- raw-data access versus vendor-fused orientation, including covariance and reset behaviour.

The selected device SHALL be validated while stationary, under actuator vibration, during known rotations, and after power cycles. Repository synthetic noise values SHALL NOT be treated as measured hardware covariance.

### 7.3 Joint encoder alternatives

The team SHALL decide among integrated actuator feedback, external absolute encoders, external incremental encoders with a homing method, or a combination. The decision SHALL address:

- observability after power cycle;
- resolution, accuracy, repeatability, latency, update rate, and missed-frame detection;
- measurement before or after gearbox/compliance;
- cable routing and environmental/mechanical exposure; and
- how disagreement between actuator and external feedback triggers a fault.

### 7.4 Foot-contact alternatives

No contact-sensing method is confirmed. The options below are alternatives, and multiple sources MAY be fused.

| Alternative | Advantages | Limitations / validation needed | Recommended stage |
| --- | --- | --- | --- |
| Mechanical switch | Simple, inexpensive, direct binary indication | Adds mechanisms and wiring at the foot; bounce, preload, durability, and terrain sensitivity | Early binary-contact prototype |
| Force-sensitive resistor, load cell, or force sensor | Measures contact magnitude and can support load distribution | Packaging, overload protection, drift, hysteresis, amplification, and multi-axis interpretation | Fixture evaluation before full deployment |
| Actuator current/torque inference | No foot wiring when current is already measured | Confounded by acceleration, friction, gearbox losses, saturation, temperature, and closed-chain load sharing | Supplemental signal after actuator characterization |
| Kinematic/dynamic inference | No additional hardware; can use joint and IMU residuals | Model-dependent and weakest during slip, impact, model error, or poor encoder observability | Simulation research, then supplemental hardware estimate |

Contact selection is a **decision required** before contact-aware state estimation or gait stabilization is accepted. Regardless of method, the system SHALL define contact/no-contact thresholds, hysteresis/debounce, latency, invalid/stuck detection, calibration, and ground-truth test method.

## 8. Safety requirements

### 8.1 Safety architecture

| ID | Requirement |
| --- | --- |
| SAF-01 | A task-specific hazard analysis SHALL cover unexpected motion, pinch/crush/shear points, falls, stored energy, sharp/hot parts, battery/fire hazards, electrical shorts, tether hazards, software faults, communications loss, and recovery after a fault. |
| SAF-02 | A clearly reachable, latching hardware e-stop SHALL remove or inhibit actuator energy independently of application software, ROS 2, wireless communications, and the companion computer. Its reset SHALL NOT automatically resume motion. Exact circuitry is a **decision required** after the power architecture is known. |
| SAF-03 | A separate software stop MAY command a controlled stop, but SHALL NOT be described as the hardware e-stop. Both paths SHALL be tested. |
| SAF-04 | Actuator enable SHALL require healthy power, communications, configuration, calibration compatibility, watchdog, and sensor/fault status plus deliberate operator action. Enable SHALL be denied with an explicit reason when any prerequisite fails. |
| SAF-05 | A low-level watchdog SHALL disable commands or actuator power when commands become stale, the host disconnects, framing fails repeatedly, or the low-level loop stalls. The timeout is **TBD**; an initial evaluation range of 50–200 ms is **proposed** and must be justified against stopping dynamics and link latency. |
| SAF-06 | Joint position, velocity, acceleration/slew, effort/current, voltage, and temperature limits SHALL be enforced as close to the actuators as practical. Host-side checks SHALL be defence in depth, not the only protection. |
| SAF-07 | Limits SHALL be stage-specific: narrower for first motion, then widened only from evidence. Invalid numbers, malformed lengths/names, out-of-order frames, stale timestamps, and commands outside limits SHALL be rejected and logged. |
| SAF-08 | Startup, reset, shutdown, brownout, disconnect, and e-stop SHALL have deterministic safe states. No actuator command SHALL be applied merely because a process restarted or an old message remained queued. |
| SAF-09 | The test area SHALL have an exclusion zone, a designated operator, a test caller, an e-stop operator when separate, an agreed abort word, and a pre-test briefing for any load-bearing or ground-motion test. |
| SAF-10 | The robot SHALL be supported during early multi-actuator tests so power loss cannot create an uncontrolled fall. Hands and tools SHALL remain outside moving linkages while actuator energy is enabled. |

### 8.2 Proposed enable-state model

The implementation SHOULD expose an explicit state machine equivalent to:

1. `POWER_OFF` — no actuator energy.
2. `LOGIC_SAFE` — controller and sensors available; actuator output inhibited.
3. `ARMED` — checks pass; actuator output still inhibited pending deliberate enable.
4. `ENABLED` — commands accepted within the limits for the current test stage.
5. `FAULT_LATCHED` — output inhibited; cause recorded; deliberate reset and re-check required.

Names and decomposition are a **decision required**, but implicit enable through topic traffic is not acceptable.

### 8.3 Bring-up procedure requirements

Every powered procedure SHALL contain:

- configuration/calibration revision, hardware revision, test stage, expected motion, limits, and pass/fail criteria;
- inspection of fixture/support, fasteners, cable routing, power, fuse, e-stop, exclusion zone, and operator roles;
- logic-only boot and telemetry check before actuator energy;
- an e-stop and watchdog functional check at the energy level appropriate to the stage;
- the smallest possible actuator set and motion envelope needed for the objective;
- live observation of power, joint state, faults, and temperatures;
- explicit abort criteria and a safe energy-removal/recovery method; and
- automatic or operator-triggered log capture with the test record.

## 9. Calibration and configuration

| ID | Requirement |
| --- | --- |
| CAL-01 | Each actuator SHALL have a stable logical joint name, leg identity, positive direction, zero reference, command conversion, measured soft limits, and calibration provenance. Numeric names used by the current prototype SHALL be migrated or mapped at the hardware boundary rather than propagated as the long-term public contract. |
| CAL-02 | Joint calibration SHALL verify repeatability from multiple approach directions and quantify backlash/hysteresis. Closed-chain calibration SHALL check consistency across coupled links rather than treating actuators independently. |
| CAL-03 | The IMU SHALL have recorded axis mapping, `base_link` transform, bias procedure, scale/misalignment calibration where supported, temperature conditions, and covariance/noise evidence. |
| CAL-04 | Contact sensing SHALL have unloaded/loaded baselines, threshold and hysteresis/debounce, overload/stuck detection, and periodic check criteria. Current-based inference SHALL account for actuator-specific friction and temperature. |
| CAL-05 | The measured robot mass, centre of mass estimate, link geometry, and available mass/inertia evidence SHALL be versioned separately from CAD exports. Simulation SHALL identify which values are measured, CAD-derived, or tuned. |
| CAL-06 | Battery voltage/current and temperature measurements SHALL be checked against traceable instruments over the intended operating range. |
| CAL-07 | Calibration data SHALL be machine-readable, schema/version controlled, tied to hardware serial/revision where available, range-checked on load, and included by identifier in every test log. |
| CAL-08 | A failed, missing, incompatible, or stale safety-critical calibration SHALL prevent actuator enable. The definition of “stale” is a **decision required** per calibration type. |

## 10. Communications and ROS 2 interface alignment

### 10.1 Logical interface contract

The simulation and hardware SHALL share a target-independent logical contract. Exact topic names and message packages are a **decision required**, but the contract SHALL include:

- commands with canonical joint names, SI units, source timestamp, sequence or freshness information, and explicit command mode;
- measured joint position and, when available, velocity, effort/current, temperature, voltage, validity, and fault state;
- `sensor_msgs/Imu`-compatible body motion with frame and covariance semantics;
- per-foot contact estimate and validity/confidence;
- power state, controller state, enable state, e-stop state, watchdog state, and latched fault reason;
- time synchronization/clock status; and
- hardware, firmware, software, model, configuration, and calibration identities.

The existing `/control_inputs` -> `/cmd_jnts` -> `/joint_states` path is a useful compatibility seam, but its numeric joint names and overloaded `JointState` command semantics are not yet a complete hardware API.

### 10.2 Transport requirements

| ID | Requirement |
| --- | --- |
| COM-01 | The MCU-to-host transport SHALL be selected after actuator bus and bandwidth inventory. Candidate transports (for example CAN/CAN-FD, USB/UART, or Ethernet) are **alternatives**, not repository facts. |
| COM-02 | The low-level protocol SHALL define framing, versioning, units, endianness, sequence handling, acquisition timestamps, validity, command expiry, checksums/CRC where the transport does not provide adequate detection, and explicit fault responses. |
| COM-03 | Measured worst-case latency, jitter, loss, restart, saturation, and disconnect behaviour SHALL meet the chosen control/watchdog budget. A nominal link-speed calculation alone is insufficient. |
| COM-04 | Wireless links MAY support supervision, visualization, and high-level commands, but SHALL NOT be the sole path for the physical e-stop. Loss of wireless communications SHALL cause the documented safe response. |
| COM-05 | ROS-domain, QoS, namespace, frame, and clock choices SHALL be documented. Safety-critical freshness SHALL not depend on an unbounded reliable queue replaying stale commands. |
| COM-06 | A diagnostic interface SHALL permit logic-only inspection and firmware recovery without actuator enable. Production operation SHALL not depend on an attached debugger. |

### 10.3 Simulation parity

To keep hardware revival aligned with simulation:

- controllers SHALL consume the same joint/foot command schema in both targets;
- simulation and hardware adapters SHALL publish the same state schema, frames, units, validity fields, and fault vocabulary where meaningful;
- target-specific APIs (PyBullet, MuJoCo, MCU protocol, actuator SDK) SHALL stay behind adapters;
- the canonical joint-name, leg-order, axis, and sign table SHALL be generated from or checked against one versioned source;
- recorded hardware commands SHALL be replayable in a safe simulation mode, and simulation-generated test vectors SHALL be usable by hardware-in-the-loop without actuator energy;
- safety clamps SHALL also be represented in simulation tests, while acknowledging that simulation does not prove physical safety; and
- model changes prompted by physical measurement SHALL retain provenance and validation results rather than being tuned without record.

## 11. Logging, diagnostics, and test records

### 11.1 Required logging

Each powered test SHALL record, as applicable:

- raw requested command, post-limit command, actuator/bus command, and command age;
- measured joint position, velocity, current/effort, voltage, temperature, validity, saturation, and fault flags;
- raw IMU acceleration/angular velocity, covariance/validity, and any derived orientation/state estimate;
- raw and filtered foot-contact signals, thresholds, confidence, and contact state;
- battery/rail voltage, current, temperature, state/faults, and e-stop/enable/watchdog transitions;
- controller loop timing, transport latency/loss/sequence errors, clock synchronization, process resets, and dropped log samples;
- transforms, estimated body state, test annotations, and video reference where useful; and
- Git revisions, firmware build, hardware revision, configuration/calibration IDs, operator, test stage, date/time, environment, fixture, and pass/fail outcome.

ROS 2 bagging is the **proposed** host-level mechanism. The low-level controller SHOULD retain a bounded diagnostic/event record sufficient to explain resets and faults that interrupt host logging.

### 11.2 Proposed rate budget

No validated physical rates exist. As a starting point for measurement only:

- a 100–250 Hz low-level command/joint-state loop;
- at least 200 Hz raw IMU acquisition when supported;
- contact sampling at the controller rate for gait use; and
- 10–50 Hz power and thermal telemetry

are **proposed evaluation ranges**, not accepted requirements. Final rates SHALL be derived from actuator dynamics, estimator/control bandwidth, link budget, timestamp quality, aliasing analysis, and measured CPU/storage load. Logs SHALL make decimation and dropped samples visible.

## 12. Verification and acceptance criteria

A requirement passes only with a linked artifact: inspection record, schematic/review, test procedure and log, measurement report, calibration file, code test, or signed decision record. “It moved” is not an acceptance criterion.

### 12.1 Cross-cutting acceptance

| ID | Acceptance criterion |
| --- | --- |
| ACC-01 | All confirmed facts used for implementation trace to repository evidence or dated physical inspection; all remaining assumptions are labelled and assigned. |
| ACC-02 | Canonical names, units, frames, signs, limits, command modes, timing, state validity, and faults are documented and tested across simulation, hardware-in-the-loop, and the physical adapter. |
| ACC-03 | Malformed, stale, duplicate/out-of-order where relevant, NaN/Inf, out-of-range, wrong-mode, and incompatible-version commands are rejected without unintended motion and with a diagnostic event. |
| ACC-04 | E-stop, watchdog, communications loss, host crash, MCU reset, brownout, sensor invalidity, over-limit, and over-current/temperature responses are tested at progressively safe energy levels before the corresponding motion gate. |
| ACC-05 | Test logs are complete enough to reconstruct command, measured response, safety transitions, configuration, and result without relying on operator memory. |
| ACC-06 | A simulation parity test sends the same bounded command fixture through simulation and hardware-in-the-loop adapters and verifies joint ordering, signs, units, freshness, limit flags, and state schema. |

### 12.2 Stage acceptance gates

| Gate | Minimum pass evidence |
| --- | --- |
| G0 — Evidence recovered | Physical inventory; photos; repository/history audit; missing-item list; named owners for critical decisions; no actuator energy applied. |
| G1 — Mechanical/passive | Structure inspected; passive motion and collision-free ranges measured; fixture/support approved; actuator and wiring identities recorded. |
| G2 — Logic and safety | Power tree reviewed; logic-only boot; sensor/bus discovery; e-stop input and output-inhibit path tested without actuator energy; watchdog/HIL fault tests pass. |
| G3 — Single actuator | Current-limited, unloaded response within a narrow range; direction/zero/feedback verified; stop, stale-command, reset, and fault behaviour logged; thermal/current baseline recorded. |
| G4 — One leg on fixture | Closed-chain geometry and limits verified; three-axis motion stays within envelope; contact alternatives evaluated; repeated trajectories meet chosen error/current/temperature criteria. |
| G5 — Full robot supported | All joints calibrated; naming/sign parity passes; coordinated sit/stand trajectory exercised while body support carries fall risk; total power and communications margin measured. |
| G6 — Static load bearing | E-stop operator and fall protection present; controlled load/unload; stable stand for a **TBD** duration; no limit, fault, thermal, wiring, or structural anomaly; safe disable/recovery demonstrated. |
| G7 — Tethered flat-floor locomotion | Low-speed start, stop, turn, and lower/recover on the defined surface; bounded tracking/contact/slip metrics; repeated trials; every abort path demonstrated at safe speed. |
| G8 — State estimation and robust gait | IMU/joint/contact validation; estimator consistency and failure detection; controlled perturbation/slip tests inside a risk-assessed envelope. |
| G9 — Navigation expansion | Perception/compute/power requirements accepted; remote stop and operating zone verified; mapping/localization/planning tested in simulation before supervised physical trials. |

Numerical pass thresholds for accuracy, repeatability, temperature, current, latency, packet loss, stand duration, speed, and trial count are all **decisions required** after G1–G4 characterization. They SHALL be fixed before the relevant acceptance run, not chosen after seeing results.

## 13. Staged hardware revival plan

### Stage 0 — Preserve evidence and resolve critical unknowns

1. Locate the robot, loose parts, batteries, charger, controller boards, firmware backups, CAD, wiring notes, and test media.
2. Photograph labels and wiring before disassembly; create the inventory and hardware revision identifier.
3. Recover actuator and power datasheets from exact part numbers.
4. Export/archive current CAD and compare link topology and dimensions with the active simplified model and older detailed exports in Git history.
5. Create owners and decision records for the critical questions in Section 14.

No actuator energy is applied in this stage.

### Stage 1 — Establish the interface and safety skeleton in simulation/HIL

1. Define canonical joints, leg order, units, frames, command modes, state validity, and fault vocabulary.
2. Implement or specify the target-independent hardware adapter boundary while retaining compatibility with the current simulation command path.
3. Build MCU/transport stubs or loopback HIL for enable state, limits, freshness, telemetry, e-stop state, and faults.
4. Add parity tests that run the same fixtures against PyBullet, planned MuJoCo work when available, and HIL.
5. Complete the power-tree and e-stop design review before fabrication or connection.

### Stage 2 — Characterize components at low energy

1. Bring up logic rails, communications, and sensors with actuator energy physically inhibited.
2. Validate timestamps, IMU frames/noise, encoder observability, power telemetry, reset causes, logging, and configuration compatibility.
3. Characterize one actuator on a fixture using a current-limited supply and narrow software limits.
4. Expand only after watchdog, e-stop/inhibit, limit, stale-command, reset, current, and thermal tests pass.

### Stage 3 — Validate one leg

1. Calibrate the three physical axes and closed-chain geometry on a rigid fixture.
2. Compare measured poses and actuator feedback with the kinematic mapper and simulation.
3. Evaluate foot-contact options using independent ground truth.
4. Feed measured geometry, range, backlash, and mass evidence back into versioned model parameters with provenance.

### Stage 4 — Integrate the supported robot

1. Calibrate all legs and verify mirrored signs/order.
2. Run coordinated low-speed postures with the robot carried by an approved support.
3. Measure peak/continuous current, rail droop, bus load, latency, loop timing, thermal rise, and structural behaviour.
4. Re-run all disable/fault paths before allowing the feet to carry body weight.

### Stage 5 — Load bearing and tethered locomotion

1. Demonstrate controlled loading, stand, lower, and fault recovery inside an exclusion zone.
2. Begin with quasistatic motions; then introduce the simplest low-speed gait already validated in simulation.
3. Compare command/state/contact bags across simulation and hardware, and update model discrepancies explicitly.
4. Require repeatable acceptance evidence before widening speed, limits, duration, or terrain.

### Stage 6 — Estimation, robust control, and eventual navigation

1. Validate IMU/joint/contact fusion and invalid-sensor behaviour in simulation, replay, and supervised physical tests.
2. Add stabilized gait work only after contact and state estimates meet chosen criteria.
3. Define perception, compute, and power needs for autonomous indoor navigation.
4. Gate every new behaviour in simulation and HIL before supervised physical use; retain a target-independent controller interface.

## 14. Open questions and required decisions

### 14.1 Blocking questions before any hardware power

| ID | Question / decision | Resolution evidence |
| --- | --- | --- |
| Q-01 | Where is the robot, who owns/controls access, and what parts, documents, batteries, chargers, boards, and tools survive? | Dated inventory and custodian |
| Q-02 | What exact mechanical revision exists, and does it match the Fusion/CAD export represented in Git? | Photos, measurements, CAD comparison |
| Q-03 | What actuator models, count, feedback, driver/interface, voltage, current, and fail behaviour are installed? | Part records, datasheets, bench plan |
| Q-04 | What battery and charger exist, and are they identifiable and serviceable? | Battery safety inspection and datasheets |
| Q-05 | What controller/driver boards were fabricated, and are schematics, PCB files, pinouts, or firmware recoverable? | Archived design package and board inspection |
| Q-06 | Who is qualified and responsible for mechanical, electrical/power, controls, and test-safety approval? | Named roles and review record |
| D-01 | Select the e-stop/power-inhibit architecture and safe behaviour for loss/reset/brownout. | Reviewed schematic and staged test plan |
| D-02 | Decide whether old batteries, boards, wiring, and actuators may be reused, refurbished, or must be replaced. | Condition evidence and rationale |

### 14.2 Decisions before coordinated actuator motion

| ID | Decision | Key inputs |
| --- | --- | --- |
| D-03 | Canonical joint names, leg order, positive axes, zeros, units, and command modes | Physical inspection, model, controller API |
| D-04 | Low-level controller and companion-compute architecture | I/O, timing, safety, maintainability, availability |
| D-05 | Actuator and host transport(s), protocol, update rate, and watchdog timeout | Bench latency/jitter/loss and stopping tests |
| D-06 | Encoder strategy and whether feedback is sufficient for load-bearing operation | Power-cycle observability, gearbox placement, accuracy |
| D-07 | IMU part, mount, interface, sampling/timestamp strategy, and calibration | Vibration/noise and estimator needs |
| D-08 | Power source, distribution, protection, rail monitoring, connector standard, and charger | Measured peak/continuous loads and safety review |
| D-09 | Mechanical repair/replacement, retention, support-fixture, soft limits, and inspection intervals | G1 inspection and component characterization |

### 14.3 Decisions before contact-aware control or navigation

| ID | Decision | Key inputs |
| --- | --- | --- |
| D-10 | Foot-contact method: switch, force sensor, current inference, kinematic inference, or fusion | Fixture ground truth, latency, robustness, packaging |
| D-11 | State-estimation architecture, frames, covariances, invalid-data and reset behaviour | IMU/joint/contact validation |
| D-12 | Quantitative gait acceptance envelope and progression rules | Supported/load-bearing characterization |
| D-13 | Perception sensors, companion compute, runtime, network, and power budget for indoor navigation | Mission scenarios after G7/G8 |
| D-14 | Data retention, privacy, and remote-operation constraints for camera/microphone or mapped environments | Intended deployment context |

## 15. Required artifacts and ownership

The revival effort SHALL maintain these versioned artifacts:

1. physical inventory and hardware revision record;
2. mechanical inspection and measured geometry/range record;
3. actuator characterization report and datasheets;
4. power tree, wiring diagram, load budget, and battery record;
5. hazard analysis, e-stop design, bring-up procedures, and test-role checklist;
6. interface control document covering joints, messages, protocol, frames, timing, validity, and faults;
7. calibration files and procedures;
8. firmware and host build/recovery instructions;
9. simulator/HIL/physical parity tests;
10. stage-specific test plans, logs, videos where useful, results, and anomaly records; and
11. decision log for every item marked **decision required**.

Each artifact SHALL name an owner and reviewer. Safety-critical artifacts SHALL be reviewed by someone other than the author before the corresponding hardware gate.

## 16. Definition of readiness for hardware work

The project is ready to begin logic-only hardware work when G0 is complete and the power/e-stop architecture has an identified owner. It is ready to energize one actuator only when G1 and G2 pass, the exact actuator and power path are known, the fixture and exclusion zone are ready, and the single-actuator procedure has fixed limits and abort criteria.

It is not ready for supported full-body motion, standing, walking, or autonomy merely because the current simulator runs or historical media show motion. Each later gate requires its own physical evidence and an explicit go/no-go decision.
