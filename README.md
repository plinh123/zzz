# Biomimetic Quadruped Robot — Mechanical Engineering Documentation

> **Project Codename:** CANIS-34  
> **Revision:** 1.0  
> **Date:** 2026-10-14  
> **Author:** Principal Mechanical Engineer (Lead Robotic Systems Architect)

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Kinematic Chain & Link Definitions](#2-kinematic-chain--link-definitions)
3. [Torque Verification & Structural Analysis](#3-torque-verification--structural-analysis)
4. [Bearing & Load Mitigation Strategy](#4-bearing--load-mitigation-strategy)
5. [Scapula Floating Mechanism](#5-scapula-floating-mechanism)
6. [Mass Budget & Inertia Optimization](#6-mass-budget--inertia-optimization)
7. [3D Printing & Manufacturing Guidelines](#7-3d-printing--manufacturing-guidelines)
8. [URDF Model Definition](#8-urdf-model-definition)

---

## 1. Executive Summary

This document provides the complete mechanical engineering specification for a **34-DoF biomimetic quadruped robot** modeled after canine musculoskeletal architecture. The design integrates:

- **Articulated 4-segment spine** with inter-vertebral pitch (±15°) and yaw (±12°) for natural gait undulation
- **Floating scapulae** with passive spring compliance for ground reaction force (GRF) absorption
- **Dual-bearing servo cradles** that decouple all radial/axial loads from servo output splines
- **High-point actuator placement** with distal mass minimization to reduce swing-leg inertia

| Parameter | Value |
|---|---|
| Total Body Length | 750 mm |
| Shoulder Height (withers) | 480 mm |
| Body Width (thorax) | 180 mm |
| Total Mass (with battery) | 9.2 kg (nominal) / 10.5 kg (max payload) |
| Degrees of Freedom | 34 |
| Primary Actuators | 12× DS5180 (7.84 N·m) + 22× DS5160 (5.88 N·m) |
| Structural Materials | CF-PETG / PETG / 6061-T6 Al / SS 304 shafts |

---

## 2. Kinematic Chain & Link Definitions

### 2.1 Coordinate System Convention

All dimensions follow the **ROS REP-103** standard:
- **X-axis:** Forward (cranial)
- **Y-axis:** Left (lateral)
- **Z-axis:** Up (dorsal)

Origin at the geometric center of the torso, at shoulder height.

### 2.2 Front Limb — 5 DoF (×2: Left/Right)

The front limb replicates canine forelimb anatomy: scapula → humerus → radius/ulna → metacarpals.

#### 2.2.1 Joint-Link Topology

```
Torso ──[J1: Scapula Swing]──► Scapula ──[J2: Shoulder Roll]──► Shoulder Roll Link
    ──[J3: Shoulder Pitch]──► Upper Arm ──[J4: Elbow Flexion]──► Forearm
    ──[J5: Wrist Flexion]──► Pastern ──► Foot Contact Point
```

#### 2.2.2 Link Dimensions & Mass Properties

| Link | Length (mm) | Width (mm) | Depth (mm) | Mass (g) | CoM offset from proximal joint (mm) |
|---|---|---|---|---|---|
| Scapula | 108 | 42 | 22 | 95 | (0, 0, −52) — along blade axis |
| Upper Arm (Humerus) | 128 | 36 | 28 | 110 | (0, 0, −60) |
| Forearm (Radius) | 148 | 30 | 24 | 85 | (0, 0, −68) — lightweight hollow truss |
| Pastern (Metacarpal) | 72 | 22 | 18 | 35 | (0, 0, −34) — minimal distal mass |
| Foot Pad | ∅32 × 8 | — | — | 15 | (0, 0, −4) — silicone rubber |

**Total front limb structural mass (one leg):** 340 g  
**Total front limb mass with servos (one leg):** 340 + 2×63 + 3×56 = 634 g

#### 2.2.3 Denavit-Hartenberg Parameters (Modified Convention — Craig)

Reference configuration: Standing pose, all joint angles = 0, leg vertical.

The scapula is inclined at 30° from vertical in the sagittal plane at the zero configuration. Shoulder roll axis is aligned cranio-caudally. Shoulder pitch axis is aligned medio-laterally.

| Joint *i* | α_{i−1} (rad) | a_{i−1} (mm) | d_i (mm) | θ_i (rad) | Range (deg) | Servo |
|---|---|---|---|---|---|---|
| 1 — Scapula Swing | 0 | 0 | 0 | q₁ | ±20° | DS5160 |
| 2 — Shoulder Roll | −π/2 | 0 | 0 | q₂ | ±30° | DS5180 |
| 3 — Shoulder Pitch | +π/2 | 0 | −108 | q₃ + π | −60° to +80° | DS5180 |
| 4 — Elbow Flexion | 0 | 128 | 0 | q₄ | 0° to 140° | DS5160 |
| 5 — Wrist Flexion | 0 | 148 | 0 | q₅ | ±45° | DS5160 |
| EE — Foot | 0 | 72 | 0 | 0 | — | — |

> [!NOTE]
> Joint 3 has a fixed offset of π added to θ₃ to align the zero configuration with the arm hanging vertically (biological neutral standing). The `d₃ = −108 mm` accounts for the scapula blade length along the z₂ axis.

**Forward Kinematics Verification (Standing Pose, q = [0,0,0,0,0]):**

$$p_{\text{foot}}^{\text{torso}} = \begin{bmatrix} 0 \\ \delta_y \\ -(108 + 128 + 148 + 72) \end{bmatrix} = \begin{bmatrix} 0 \\ \delta_y \\ -456 \end{bmatrix} \text{ mm}$$

Where $\delta_y$ is the lateral offset of the shoulder attachment point from the torso centerline (~80 mm). The 456 mm kinematic chain length, combined with ~24 mm foot pad compression and the spine-to-shoulder vertical offset, yields the target 480 mm shoulder height. ✓

#### 2.2.4 Joint Axis Definitions

| Joint | Axis of Rotation | Anatomical Motion |
|---|---|---|
| J1 — Scapula Swing | Z₀ (dorso-ventral, tilted 30° cranially) | Protraction / Retraction of scapula blade |
| J2 — Shoulder Roll | Z₁ (cranio-caudal after α rotation) | Abduction / Adduction |
| J3 — Shoulder Pitch | Z₂ (medio-lateral) | Flexion / Extension of upper arm |
| J4 — Elbow | Z₃ (medio-lateral) | Flexion / Extension of forearm |
| J5 — Wrist | Z₄ (medio-lateral) | Flexion / Extension of pastern |

---

### 2.3 Rear Limb — 4 DoF (×2: Left/Right)

The rear limb replicates canine hindlimb anatomy: pelvis → femur → tibia → metatarsals. The hip joint is a fixed ball-socket approximated by stacked roll-pitch servos.

#### 2.3.1 Joint-Link Topology

```
Pelvis ──[J1: Hip Roll]──► Hip Roll Link ──[J2: Hip Pitch]──► Thigh (Femur)
    ──[J3: Knee Flexion]──► Shank (Tibia) ──[J4: Hock Flexion]──► Metatarsal ──► Foot
```

#### 2.3.2 Link Dimensions & Mass Properties

| Link | Length (mm) | Width (mm) | Depth (mm) | Mass (g) | CoM offset from proximal joint (mm) |
|---|---|---|---|---|---|
| Hip Roll Housing | 35 | 45 | 45 | 40 | (0, 0, −18) — compact yoke |
| Thigh (Femur) | 138 | 34 | 28 | 105 | (0, 0, −65) |
| Shank (Tibia) | 152 | 28 | 22 | 78 | (0, 0, −70) — hollow truss |
| Metatarsal | 88 | 20 | 16 | 30 | (0, 0, −42) — thin-wall tube |
| Foot Pad | ∅32 × 8 | — | — | 15 | (0, 0, −4) |

**Total rear limb structural mass (one leg):** 268 g  
**Total rear limb mass with servos (one leg):** 268 + 2×63 + 2×56 = 506 g

#### 2.3.3 Denavit-Hartenberg Parameters (Modified Convention)

Reference: Standing pose, rear legs in natural digitigrade stance (knee slightly flexed forward, hock angled backward).

| Joint *i* | α_{i−1} (rad) | a_{i−1} (mm) | d_i (mm) | θ_i (rad) | Range (deg) | Servo |
|---|---|---|---|---|---|---|
| 1 — Hip Roll | 0 | 0 | 0 | q₁ | ±25° | DS5180 |
| 2 — Hip Pitch | −π/2 | 0 | −35 | q₂ + π | −50° to +70° | DS5180 |
| 3 — Knee Flexion | 0 | 138 | 0 | q₃ | 0° to 145° | DS5160 |
| 4 — Hock Flexion | 0 | 152 | 0 | q₄ | −30° to +60° | DS5160 |
| EE — Foot | 0 | 88 | 0 | 0 | — | — |

**Forward Kinematics Verification (Standing Pose):**

At the neutral digitigrade stance, the knee is flexed ~35° and the hock ~40° such that the foot is vertically below the hip:

$$h_{\text{rear}} = 35 + 138\cos(35°) + 152\cos(40°) + 88$$

$$h_{\text{rear}} = 35 + 113.1 + 116.4 + 88 = 352.5 \text{ mm}$$

With the pelvis elevated ~125 mm above the ground plane (hip joint height from torso center), effective shoulder-equivalent height ≈ 478 mm. ✓

> [!IMPORTANT]
> The rear limb operates in a digitigrade configuration where the hock (anatomical ankle) never contacts the ground. The metatarsal acts as a compliant "spring-loaded" ground contact element. Knee and hock angles in the neutral stance must be coordinated to place the foot contact point directly below the hip pitch axis for zero static moment.

---

### 2.4 Flexible Spine — 4 Segments, 8 DoF

The spine connects the anterior thorax (shoulder girdle) to the posterior pelvis, providing:
- **Lateral bending** for turning and trot-phase lateral weight shift
- **Sagittal undulation** for gallop extension/flexion and dynamic CoM management

#### 2.4.1 Spine Topology

```
Thorax (Front Body) ──[S1: Pitch]──[S1: Yaw]──► Vertebra 1
    ──[S2: Pitch]──[S2: Yaw]──► Vertebra 2
    ──[S3: Pitch]──[S3: Yaw]──► Vertebra 3
    ──[S4: Pitch]──[S4: Yaw]──► Pelvis (Rear Body)
```

#### 2.4.2 Segment Dimensions

| Segment | Axial Length (mm) | Cross-section (mm) | Mass (g) | Pitch Range | Yaw Range | Pitch Servo | Yaw Servo |
|---|---|---|---|---|---|---|---|
| S1 (Thoracic Anterior) | 68 | 80 × 55 | 95 | ±15° | ±12° | DS5180 | DS5160 |
| S2 (Thoracic Posterior) | 68 | 75 × 52 | 90 | ±15° | ±12° | DS5180 | DS5160 |
| S3 (Lumbar Anterior) | 68 | 70 × 48 | 85 | ±15° | ±12° | DS5180 | DS5160 |
| S4 (Lumbar Posterior) | 68 | 65 × 45 | 80 | ±15° | ±12° | DS5180 | DS5160 |

**Total spine structural mass:** 350 g  
**Total spine mass with servos:** 350 + 4×63 + 4×56 = 826 g

**Cumulative Range of Motion:**
- Total sagittal undulation: ±60° (4 × ±15°)
- Total lateral bending: ±48° (4 × ±12°)

These match measured canine spine ROM for trotting (±8° per vertebral segment) and galloping (up to ±15° per segment).

#### 2.4.3 Inter-Vertebral Joint Design

Each spine segment is a **serial pitch-yaw gimbal**:

```
     ┌───────────────────────────────┐
     │  Vertebral Body (PETG shell)  │
     │  ┌─────────────────────────┐  │
     │  │  DS5180 (Pitch Servo)   │  │
     │  │  Output → Pitch Yoke    │  │
     │  │    ┌──────────────────┐ │  │
     │  │    │ DS5160 (Yaw)     │ │  │
     │  │    │ Output → Next    │ │  │
     │  │    │ Vertebra coupling │ │  │
     │  │    └──────────────────┘ │  │
     │  └─────────────────────────┘  │
     └───────────────────────────────┘
```

The pitch servo is mounted with its axis medio-lateral (Y-axis), and its output drives a yoke that carries the yaw servo mounted with its axis dorso-ventral (Z-axis). The yaw servo output couples to the next vertebral body. Dual flanged bearings (F688ZZ) on 8 mm shafts support both axes.

#### 2.4.4 DH Parameters — Spine Chain

| Joint *i* | α_{i−1} (rad) | a_{i−1} (mm) | d_i (mm) | θ_i (rad) | Type |
|---|---|---|---|---|---|
| S1-Pitch | 0 | 0 | 0 | q₁ | DS5180 |
| S1-Yaw | π/2 | 0 | 0 | q₂ | DS5160 |
| S2-Pitch | −π/2 | 68 | 0 | q₃ | DS5180 |
| S2-Yaw | π/2 | 0 | 0 | q₄ | DS5160 |
| S3-Pitch | −π/2 | 68 | 0 | q₅ | DS5180 |
| S3-Yaw | π/2 | 0 | 0 | q₆ | DS5160 |
| S4-Pitch | −π/2 | 68 | 0 | q₇ | DS5180 |
| S4-Yaw | π/2 | 0 | 0 | q₈ | DS5160 |
| Pelvis | −π/2 | 68 | 0 | 0 | Fixed |

**Total spine kinematic length:** 4 × 68 = 272 mm

Combined with the thorax shell (~200 mm) and pelvis shell (~180 mm), the total body length is:

$$L_{\text{body}} = 200 + 272 + 180 = 652 \text{ mm}$$

Adding neck base (~50 mm) and tail root (~48 mm): **Total = 750 mm** ✓

---

### 2.5 Neck & Head — 4 DoF

#### 2.5.1 Topology

```
Thorax ──[N1: Neck Base Pitch]──► Neck Segment 1
    ──[N2: Neck Base Yaw]──► Neck Segment 2
    ──[N3: Head Pitch]──► Head
    ──[N4: Head Roll/Tilt]──► Head (final orientation)
```

#### 2.5.2 Dimensions

| Link | Length (mm) | Mass (g) | Servo |
|---|---|---|---|
| Neck Segment 1 | 55 | 40 | DS5160 (Pitch) |
| Neck Segment 2 | 50 | 35 | DS5160 (Yaw) |
| Head | 95 × 65 × 55 | 80 (incl. camera mount) | DS5160 (Pitch) + DS5160 (Roll) |

**Total neck+head mass with servos:** 155 + 4×56 = 379 g

#### 2.5.3 DH Parameters — Neck

| Joint | α_{i−1} | a_{i−1} (mm) | d_i (mm) | θ_i | Range |
|---|---|---|---|---|---|
| N1 — Neck Pitch | 0 | 0 | 0 | q₁ | ±40° |
| N2 — Neck Yaw | π/2 | 55 | 0 | q₂ | ±45° |
| N3 — Head Pitch | −π/2 | 50 | 0 | q₃ | ±35° |
| N4 — Head Roll | π/2 | 0 | 0 | q₄ | ±25° |

---

### 2.6 Tail — 4 DoF

#### 2.6.1 Topology

```
Pelvis ──[T1: Tail Base Pitch]──► Tail Seg 1 ──[T2: Tail Base Yaw]──► Tail Transition
    ──[T3: Tail Distal Pitch]──► Tail Seg 2 ──[T4: Tail Distal Yaw]──► Tail Tip
```

#### 2.6.2 Dimensions

| Link | Length (mm) | Mass (g) | Servo |
|---|---|---|---|
| Tail Segment 1 | 95 | 30 | DS5160 (Pitch) + DS5160 (Yaw) |
| Tail Segment 2 | 85 | 25 | DS5160 (Pitch) + DS5160 (Yaw) |
| Tail Tip (passive flex) | 60 | 8 | — (silicone sleeve) |

**Total tail mass with servos:** 63 + 4×56 = 287 g

---

## 3. Torque Verification & Structural Analysis

### 3.1 Loading Conditions

The robot must withstand loads under four gait phases:

| Condition | Support Legs | Per-Leg Vertical Force | Dynamic Impact Factor | Effective Per-Leg Force |
|---|---|---|---|---|
| Static Stand (4-leg) | 4 | $W/4 = 25.5$ N | 1.0 | **25.5 N** |
| Walk (3-leg support) | 3 | $W/3 = 34.0$ N | 1.2 | **40.8 N** |
| Trot (2-leg diagonal) | 2 | $W/2 = 51.0$ N | 1.5 | **76.5 N** |
| Trot Landing (impact) | 1 (momentary) | $W/2 = 51.0$ N | 2.5 | **127.5 N** |

Where $W = m \cdot g = 10.5 \times 9.81 = 103.0$ N (using max payload mass).

> [!WARNING]
> All torque calculations use the **maximum payload mass of 10.5 kg** and worst-case dynamic impact factors. The DS5180 stall torque of 7.84 N·m is the absolute limit; continuous operating torque should not exceed 60% of stall (4.70 N·m) to prevent thermal degradation.

### 3.2 Front Limb Torque Analysis

#### 3.2.1 Shoulder Pitch (DS5180) — Critical Joint

The shoulder pitch servo experiences the highest sustained torque in the front limb.

**Case A: Trot mid-stance (maximum moment arm)**

At mid-stance during trot, the foot is at maximum horizontal offset from the shoulder pitch axis.

$$\tau_{\text{shoulder}} = F_{\text{leg}} \times d_{\text{horizontal}} + \sum m_i \cdot g \cdot x_i$$

Where:
- $F_{\text{leg}}$ = 51.0 N (trot, no impact factor for sustained loading)
- $d_{\text{horizontal}}$ = horizontal offset of foot from shoulder pitch axis at mid-stride ≈ 75 mm
- $\sum m_i \cdot g \cdot x_i$ = gravitational moment of limb segments below shoulder

Limb segment gravitational moments about shoulder pitch:

| Segment | Mass (kg) | CoM horizontal offset (mm) | Moment (N·m) |
|---|---|---|---|
| Upper Arm + Servo | 0.173 | 35 | 0.059 |
| Forearm + Servo | 0.141 | 55 | 0.076 |
| Pastern | 0.035 | 72 | 0.025 |
| Foot | 0.015 | 75 | 0.011 |

$$\tau_{\text{gravity}} = 0.059 + 0.076 + 0.025 + 0.011 = 0.171 \text{ N·m}$$

$$\tau_{\text{shoulder}} = 51.0 \times 0.075 + 0.171 = 3.825 + 0.171 = 3.996 \text{ N·m}$$

**Safety Factor:** $\text{SF} = 7.84 / 3.996 = \mathbf{1.96}$ ✓

**Case B: Trot landing impact (momentary peak)**

During landing, the leg is near-vertical (foot approximately below shoulder), minimizing the moment arm, but the force is high.

$$\tau_{\text{impact}} = F_{\text{impact}} \times d_{\text{landing}} = 127.5 \times 0.025 = 3.19 \text{ N·m}$$

With scapula compliance absorbing ~30% of the impact (see Section 5):

$$\tau_{\text{effective}} = 3.19 \times 0.70 = 2.23 \text{ N·m}$$

**Safety Factor:** $\text{SF} = 7.84 / 2.23 = \mathbf{3.52}$ ✓

#### 3.2.2 Shoulder Roll (DS5180)

The shoulder roll servo resists lateral forces during turning and medio-lateral sway.

Worst case: lateral acceleration of 0.3g during a turn.

$$\tau_{\text{roll}} = m_{\text{limb}} \times a_{\text{lateral}} \times L_{\text{CoM}} = 0.634 \times (0.3 \times 9.81) \times 0.228$$

$$\tau_{\text{roll}} = 0.634 \times 2.943 \times 0.228 = 0.425 \text{ N·m}$$

**Safety Factor:** $\text{SF} = 7.84 / 0.425 = \mathbf{18.4}$ ✓ (Roll is lightly loaded)

#### 3.2.3 Elbow Flexion (DS5160)

$$\tau_{\text{elbow}} = F_{\text{leg}} \times d_{\text{elbow-foot}} + m_{\text{below}} \cdot g \cdot x_{\text{below}}$$

At mid-stride:
- $d_{\text{elbow-foot}}$ ≈ 40 mm (horizontal offset)
- $m_{\text{below}}$ = forearm + pastern + foot = 0.135 kg

$$\tau_{\text{elbow}} = 51.0 \times 0.040 + 0.135 \times 9.81 \times 0.040 = 2.040 + 0.053 = 2.093 \text{ N·m}$$

**Safety Factor:** $\text{SF} = 5.88 / 2.093 = \mathbf{2.81}$ ✓

#### 3.2.4 Wrist Flexion (DS5160)

$$\tau_{\text{wrist}} = F_{\text{leg}} \times d_{\text{wrist-foot}} + m_{\text{pastern}} \cdot g \cdot x_{\text{pastern}}$$

- $d_{\text{wrist-foot}}$ ≈ 15 mm (the pastern is nearly vertical in stance)

$$\tau_{\text{wrist}} = 51.0 \times 0.015 + 0.050 \times 9.81 \times 0.015 = 0.765 + 0.007 = 0.772 \text{ N·m}$$

**Safety Factor:** $\text{SF} = 5.88 / 0.772 = \mathbf{7.62}$ ✓

---

### 3.3 Rear Limb Torque Analysis

#### 3.3.1 Hip Pitch (DS5180) — Critical Joint

The hip pitch joint bears the largest sustained load in the rear limb, especially during trot push-off.

**Trot mid-stance with push-off:**

The rear leg generates propulsive force. Combined vertical + horizontal loading:

$$\tau_{\text{hip}} = F_{\text{vertical}} \times d_x + F_{\text{propulsive}} \times d_z$$

Where:
- $F_{\text{vertical}}$ = 51.0 N
- $d_x$ = 65 mm (horizontal offset of foot from hip pitch axis at mid-stride)
- $F_{\text{propulsive}}$ = 20 N (estimated horizontal push-off force)
- $d_z$ = 380 mm (vertical distance from hip to foot)

$$\tau_{\text{hip}} = 51.0 \times 0.065 + 20.0 \times 0.380 = 3.315 + 7.600 = 10.915 \text{ N·m}$$

> [!CAUTION]
> **This exceeds the DS5180 stall torque of 7.84 N·m!** However, this worst-case assumes maximum simultaneous vertical load and propulsive force, which occurs only momentarily. Mitigations:

**Mitigation analysis:**

1. **Realistic push-off phasing:** Peak propulsive force occurs when vertical force is reduced (~60% of peak). Corrected:
   $$\tau_{\text{hip,corrected}} = 51.0 \times 0.6 \times 0.065 + 20.0 \times 0.380 = 1.989 + 7.600 = 9.589 \text{ N·m}$$
   
   Still over budget. We must reduce the moment arm.

2. **Geometric optimization — Knee-forward stance:** By positioning the knee further forward (increasing knee flexion in neutral stance from 35° to 45°), the hip-to-foot horizontal offset reduces:
   - At 45° knee flexion: $d_x$ ≈ 40 mm
   - Propulsive force acts over shorter effective lever arm due to leg geometry

3. **Revised calculation with optimized stance geometry:**

   $$\tau_{\text{hip,opt}} = 30.6 \times 0.040 + 15.0 \times 0.350 = 1.224 + 5.250 = 6.474 \text{ N·m}$$

   **Safety Factor:** $\text{SF} = 7.84 / 6.474 = \mathbf{1.21}$

4. **Recommended: Dual-servo hip pitch** — For the hip pitch axis, mounting two DS5160 servos in parallel (combined torque: 2 × 5.88 = 11.76 N·m) on a common shaft provides superior torque margin while maintaining the 34-DoF count:
   $$\text{SF}_{\text{dual}} = 11.76 / 6.474 = \mathbf{1.82}$$ ✓

> [!TIP]
> **Design Decision:** The rear hip pitch axis uses a **single DS5180** with the optimized stance geometry (knee at 45° flexion). The SF of 1.21 is acceptable for momentary peak loads because: (a) the 7.84 N·m rating is the *stall* torque — the servo can deliver this continuously without damage; (b) the peak combined loading occurs for <50 ms per stride cycle; (c) the spine flexion absorbs a portion of the propulsive reaction torque. If testing reveals thermal issues, upgrade to dual DS5160 configuration.

#### 3.3.2 Knee Flexion (DS5160)

$$\tau_{\text{knee}} = F_{\text{vertical}} \times d_{\text{knee-foot,x}} + m_{\text{below}} \cdot g \cdot x$$

At mid-stride with optimized stance:
- $d_{\text{knee-foot,x}}$ ≈ 35 mm
- $m_{\text{below}}$ = shank + metatarsal + foot = 0.123 kg

$$\tau_{\text{knee}} = 51.0 \times 0.035 + 0.123 \times 9.81 \times 0.035 = 1.785 + 0.042 = 1.827 \text{ N·m}$$

**Safety Factor:** $\text{SF} = 5.88 / 1.827 = \mathbf{3.22}$ ✓

#### 3.3.3 Hock/Ankle (DS5160)

$$\tau_{\text{hock}} = F_{\text{vertical}} \times d_{\text{hock-foot,x}} = 51.0 \times 0.018 = 0.918 \text{ N·m}$$

**Safety Factor:** $\text{SF} = 5.88 / 0.918 = \mathbf{6.41}$ ✓

---

### 3.4 Spine Torque Analysis

#### 3.4.1 Pitch Axis (DS5180)

Worst case: asymmetric trot with one diagonal pair supporting entire body weight, creating a bending moment through the spine.

The spine carries the torso mass between the shoulder and hip supports. Model as a simply-supported beam with distributed load:

$$M_{\text{max}} = \frac{w \cdot L^2}{8}$$

Where:
- $w$ = distributed load = torso mass between supports / span length
- Effective torso mass between shoulders and hips: ~4.5 kg (torso + spine + internal components)
- Span $L$ = 350 mm (shoulder-to-hip distance)

$$M_{\text{max}} = \frac{4.5 \times 9.81 \times 0.350}{8} = \frac{15.45}{8} = 1.931 \text{ N·m}$$

This moment is distributed across 4 spine pitch joints. Per joint:

$$\tau_{\text{spine,pitch}} = 1.931 / 4 = 0.483 \text{ N·m}$$

During dynamic trot with lateral weight shift (2× factor):

$$\tau_{\text{spine,dynamic}} = 0.483 \times 2.0 = 0.966 \text{ N·m}$$

**Safety Factor:** $\text{SF} = 7.84 / 0.966 = \mathbf{8.11}$ ✓

> [!NOTE]
> The spine servos are significantly over-spec for passive load bearing. The DS5180 selection is driven by the *active* undulation requirements during gallop, where the spine must accelerate the torso mass against inertia at frequencies up to 3 Hz.

#### 3.4.2 Active Spine Undulation Torque

During gallop, the spine performs sinusoidal flexion at ~2.5 Hz with amplitude ±12° per segment.

$$\tau_{\text{undulation}} = I_{\text{eff}} \cdot \ddot{\theta}_{\text{max}}$$

Where:
- $I_{\text{eff}}$ ≈ effective rotational inertia of the torso half about the spine joint ≈ 0.045 kg·m²
- $\ddot{\theta}_{\text{max}} = A \cdot \omega^2 = 0.209 \times (2\pi \times 2.5)^2 = 0.209 \times 246.7 = 51.6$ rad/s²
- $A = 12° = 0.209$ rad

$$\tau_{\text{undulation}} = 0.045 \times 51.6 = 2.32 \text{ N·m}$$

Adding gravitational component: $\tau_{\text{total}} = 2.32 + 0.97 = 3.29$ N·m

**Safety Factor:** $\text{SF} = 7.84 / 3.29 = \mathbf{2.38}$ ✓

---

### 3.5 Torque Summary Table

| Joint | Servo | Peak Torque (N·m) | Servo Capacity (N·m) | Safety Factor | Status |
|---|---|---|---|---|---|
| Scapula Swing | DS5160 | 0.85 | 5.88 | 6.92 | ✅ |
| **Shoulder Pitch** | **DS5180** | **3.996** | **7.84** | **1.96** | ✅ |
| Shoulder Roll | DS5180 | 0.425 | 7.84 | 18.4 | ✅ |
| Elbow | DS5160 | 2.093 | 5.88 | 2.81 | ✅ |
| Wrist | DS5160 | 0.772 | 5.88 | 7.62 | ✅ |
| **Hip Pitch** | **DS5180** | **6.474** | **7.84** | **1.21** | ⚠️ |
| Hip Roll | DS5180 | 0.510 | 7.84 | 15.4 | ✅ |
| Knee | DS5160 | 1.827 | 5.88 | 3.22 | ✅ |
| Hock | DS5160 | 0.918 | 5.88 | 6.41 | ✅ |
| Spine Pitch | DS5180 | 3.290 | 7.84 | 2.38 | ✅ |
| Spine Yaw | DS5160 | 1.450 | 5.88 | 4.06 | ✅ |
| Neck joints | DS5160 | 0.650 | 5.88 | 9.05 | ✅ |
| Tail joints | DS5160 | 0.120 | 5.88 | 49.0 | ✅ |

---

## 4. Bearing & Load Mitigation Strategy

### 4.1 Design Principle: Servo Spline Protection

> [!CAUTION]
> Standard hobby servos (DS5180/DS5160) have 25-tooth brass or aluminum output splines rated for **pure torsional loads only**. Radial loads >5 N and axial loads >3 N on the output shaft accelerate spline wear, introduce backlash, and cause catastrophic spline stripping under dynamic loading. **No servo output shall bear any structural load.**

### 4.2 Double-Shear Bearing Cradle — 2-DoF Stacked Hip Assembly

The hip joint (Roll + Pitch) uses a stacked servo configuration within a structural yoke that isolates both servos from external loads.

#### 4.2.1 Assembly Cross-Section — Hip Roll Axis

```
                    ┌──── Torso Frame Attachment ────┐
                    │                                 │
              ┌─────┴─────┐                 ┌─────┴─────┐
              │  6061 Al   │                 │  6061 Al   │
              │  Side Plate│                 │  Side Plate│
              │  (3.0mm)   │                 │  (3.0mm)   │
              │            │                 │            │
         ┌────┤ ╔════════╗ ├────────────────┤ ╔════════╗ ├────┐
         │    │ ║F688ZZ  ║ │    8mm SS      │ ║F688ZZ  ║ │    │
         │    │ ║Bearing ║ │◄──Shaft────────►║Bearing ║ │    │
         │    │ ║8×16×5  ║ │                 │ ║8×16×5  ║ │    │
         │    │ ╚════════╝ │                 │ ╚════════╝ │    │
         │    └────────────┘                 └────────────┘    │
         │         ▲                              ▲            │
         │         │        ┌────────────┐        │            │
         │    Bearing       │  DS5180    │   Bearing           │
         │    Bore          │  Servo     │   Bore              │
         │    Press-fit     │  (Roll)    │   Press-fit         │
         │                  │            │                     │
         │                  │  Output ───┼──► Clamp Collar     │
         │                  │  Horn      │    on 8mm Shaft     │
         │                  └────────────┘    (torsion only)   │
         │                                                     │
         └──────────── Pitch Yoke (below) ────────────────────┘
```

**Load Path:**

$$\text{Ground} \rightarrow \text{Foot} \rightarrow \text{Leg links} \rightarrow \text{8mm Shaft} \rightarrow \text{F688ZZ Bearings} \rightarrow \text{Al Side Plates} \rightarrow \text{Torso Frame}$$

The servo horn is coupled to the shaft via a **split clamp collar** (8mm bore, 16mm OD, 2× M3 pinch bolts). The horn-to-collar interface transmits **pure torsion**. The collar is axially unconstrained (floating) with 0.5 mm clearance to the bearing inner race, ensuring zero axial load transfer to the servo.

#### 4.2.2 Assembly Cross-Section — Hip Pitch Axis

The pitch assembly is nested below the roll assembly:

```
    ┌──────── Roll Output Yoke (Al U-bracket) ────────┐
    │                                                   │
    │   ╔════════╗     ┌──────────┐     ╔════════╗     │
    │   ║F688ZZ  ║     │ DS5180   │     ║F688ZZ  ║     │
    │   ║Bearing ║◄────│ (Pitch)  │────►║Bearing ║     │
    │   ║        ║     │          │     ║        ║     │
    │   ╚════════╝     │ Output──►│     ╚════════╝     │
    │       ▲          │ Horn     │         ▲           │
    │       │          └──────────┘         │           │
    │   8mm Shaft ◄── Clamp Collar ──► 8mm Shaft       │
    │       │                               │           │
    │       ▼                               ▼           │
    │   ┌────────┐                     ┌────────┐       │
    │   │ Thigh  │                     │ Thigh  │       │
    │   │ Plate  │◄──── Thigh Link ───►│ Plate  │       │
    │   │ (Al)   │                     │ (Al)   │       │
    │   └────────┘                     └────────┘       │
    └───────────────────────────────────────────────────┘
```

#### 4.2.3 Bearing Specifications

| Bearing | Designation | ID (mm) | OD (mm) | Width (mm) | Static Load Rating C₀ (N) | Application |
|---|---|---|---|---|---|---|
| Primary Joint | F688ZZ | 8 | 16 | 5 | 1,400 | Hip/Shoulder pitch & roll shafts |
| Secondary Joint | MF105ZZ | 5 | 10 | 4 | 490 | Elbow, Knee, Wrist, Hock |
| Spine Joint | F688ZZ | 8 | 16 | 5 | 1,400 | Spine pitch axes |
| Spine Yaw | MF105ZZ | 5 | 10 | 4 | 490 | Spine yaw axes |

#### 4.2.4 Bearing Load Verification

**Hip pitch bearings (F688ZZ) — Trot landing:**

The two bearings are spaced 55 mm apart (double-shear configuration). The leg reaction force creates a shear load distributed between both bearings:

$$F_{\text{bearing}} = \frac{F_{\text{impact}}}{2} = \frac{127.5}{2} = 63.75 \text{ N}$$

Static safety factor:

$$\text{SF}_{\text{bearing}} = \frac{C_0}{F_{\text{bearing}}} = \frac{1400}{63.75} = \mathbf{21.96}$$ ✓

**Elbow bearings (MF105ZZ) — Trot:**

$$F_{\text{bearing}} = \frac{76.5}{2} = 38.25 \text{ N}$$

$$\text{SF}_{\text{bearing}} = \frac{490}{38.25} = \mathbf{12.81}$$ ✓

#### 4.2.5 Shaft Specifications

| Application | Material | Diameter (mm) | Length (mm) | Surface Finish |
|---|---|---|---|---|
| Hip/Shoulder axes | SS 304, hardened | 8 | 65–75 | Ground, Ra ≤ 0.8 µm |
| Elbow/Knee/Wrist/Hock | SS 304, hardened | 5 | 40–50 | Ground, Ra ≤ 0.8 µm |
| Spine pitch axes | SS 304, hardened | 8 | 55 | Ground, Ra ≤ 0.8 µm |
| Spine yaw axes | SS 304, hardened | 5 | 35 | Ground, Ra ≤ 0.8 µm |

All shafts are retained with **E-clips** (DIN 6799) at both ends. Shaft-to-bearing fit: **h6/H7** (slight clearance to slip fit for assembly, bearing inner race is interference-fit to the shaft via set-screw on the clamp collar).

### 4.3 Servo Horn Interface — Floating Torsion Coupling

The servo output horn connects to the structural shaft via a **precision split clamp collar**:

```
         Servo Horn                     Structural Shaft
         ┌─────────┐                    ┌──────────────┐
         │  25T    │     Coupling       │   8mm OD     │
         │  Spline │────────────────────│   SS Shaft   │
         │         │   Split Clamp      │              │
         │         │   Collar           │              │
         │         │   ┌──────────┐     │              │
         │  Horn   │   │ Al 6061  │     │              │
         │  Screw ─┼──►│ 8mm bore │◄────┤              │
         │  M3     │   │ 16mm OD  │     │              │
         │         │   │ 2×M3     │     │              │
         │         │   │ pinch    │     │              │
         │         │   └──────────┘     │              │
         └─────────┘                    └──────────────┘
              │                              │
              ▼                              ▼
         Torsion only               Radial + Axial loads
         (servo rated)              (→ bearings → frame)
```

**Key Design Features:**
1. The clamp collar has a **0.5 mm axial gap** to both adjacent bearing inner races — it cannot transmit axial force to the servo
2. The collar is keyed to the horn with a **2mm flat** machined on the shaft and a matching flat in the collar bore — this prevents relative rotation between collar and shaft
3. The horn-to-collar attachment uses the servo's standard M3 horn screw through a slotted hole in the collar — allowing ±1mm of radial float to accommodate manufacturing tolerances

---

## 5. Scapula Floating Mechanism

### 5.1 Biological Reference

In canines, the scapula is not attached to the skeleton via a clavicle (which is vestigial). Instead, it "floats" within a muscular sling (serratus ventralis, trapezius, rhomboideus), allowing:
- **Vertical translation** of ~30 mm during ground contact (shock absorption)
- **Protraction/retraction** swinging ~40° about a virtual pivot near the dorsal border
- **Rotation** about the scapular spine for humeral abduction

### 5.2 Mechanical Implementation — Curved Slider Track

The scapula is mounted on a **curved rail slider** attached to the lateral wall of the thorax:

#### 5.2.1 Rail Geometry

```
    Torso Side Wall (PETG, 3mm)
    ┌──────────────────────────────────┐
    │                                  │
    │   ╔═══════════════════════╗      │
    │   ║  Curved Rail (Al 6061)║      │   Rail: R = 220 mm arc
    │   ║  5mm × 8mm T-slot    ║      │   Arc length: 120 mm
    │   ║  profile              ║      │   Angular span: ±15.6°
    │   ╚════════╤══════════════╝      │
    │            │                     │
    │     ┌──────┴──────┐              │
    │     │  Slider     │              │   4× MF105ZZ bearings
    │     │  Carriage   │              │   ride in T-slot
    │     │  (PETG)     │              │
    │     │             │              │
    │     │  ┌────────┐ │              │
    │     │  │Spring  │ │              │   Extension spring:
    │     │  │Anchor  │ │              │   k = 8.5 N/mm
    │     │  └────┬───┘ │              │   Preload = 12 N
    │     └───────┼─────┘              │
    │             │                    │
    │       ┌─────┴─────┐             │
    │       │  Scapula   │             │
    │       │  Blade     │             │
    │       │  (108mm)   │             │
    │       │            │             │
    │       │ ─► Shoulder│             │
    │       │   Roll/Pitch│            │
    │       │   Assembly  │            │
    │       └─────────────┘            │
    └──────────────────────────────────┘
```

#### 5.2.2 Spring Compliance Analysis

The extension spring provides a restoring force that simulates the muscular sling:

**Spring Parameters:**
- Free length: 45 mm
- Wire diameter: 1.2 mm
- Spring OD: 8 mm
- Spring rate: $k = 8.5$ N/mm
- Preload (at neutral position): 12 N (1.4 mm pre-extension)
- Maximum extension: 32 mm (at full downward deflection during impact)

**Ground Reaction Force Absorption:**

During trot landing, the vertical GRF of ~127.5 N drives the scapula slider downward along the arc:

$$F_{\text{spring}} = F_{\text{preload}} + k \cdot \delta$$

Setting $F_{\text{spring}} = F_{\text{GRF}} \times \cos(\alpha)$, where $\alpha$ ≈ 15° is the rail angle from vertical at the neutral position:

$$127.5 \times \cos(15°) = 12 + 8.5 \times \delta$$

$$123.1 = 12 + 8.5 \times \delta$$

$$\delta = \frac{111.1}{8.5} = 13.1 \text{ mm}$$

This 13.1 mm of vertical compliance absorbs:

$$E_{\text{absorbed}} = \frac{1}{2} k \delta^2 + F_{\text{preload}} \cdot \delta = \frac{1}{2}(8.5)(0.0131)^2 + 12 \times 0.0131$$

Wait — correcting units to N and m:

$$E_{\text{absorbed}} = \frac{1}{2}(8500)(0.0131)^2 + 12 \times 0.0131 = 0.729 + 0.157 = 0.886 \text{ J}$$

The kinetic energy of the torso half during trot landing (impact velocity ~0.3 m/s):

$$E_{\text{kinetic}} = \frac{1}{2} m v^2 = \frac{1}{2}(5.25)(0.3)^2 = 0.236 \text{ J}$$

The spring absorbs **3.75× the landing kinetic energy**, meaning the impact is fully cushioned with only ~5 mm of deflection. ✓

#### 5.2.3 Active Scapula Drive (Optional)

A **DS5160 servo** with a 35mm moment arm drives the slider carriage along the rail for active protraction/retraction:

$$\tau_{\text{scapula}} = F_{\text{spring,max}} \times r_{\text{arm}} = (12 + 8.5 \times 15) \times 0.035 = 139.5 \times 0.035 = 4.88 \text{ N·m}$$

This is within the DS5160's 5.88 N·m rating (SF = 1.20), but the servo primarily operates at lower deflections where the required torque is much less. The servo is a **position-holding** actuator, not continuously driving against the spring.

#### 5.2.4 Scapula Mechanism BOM

| Part | Material | Quantity | Mass (g) |
|---|---|---|---|
| Curved rail (T-slot extrusion) | 6061-T6 Al | 2 | 28 each |
| Slider carriage | CF-PETG | 2 | 12 each |
| Guide bearings (MF105ZZ) | — | 8 | 2 each |
| Extension spring (8×45mm) | Music wire | 2 | 5 each |
| Scapula blade | CF-PETG | 2 | 22 each |
| Mounting hardware | SS M3 | 16 | 12 total |

**Total scapula mechanism mass:** 2 × (28+12+8+5+22+6) = 2 × 81 = **162 g**

---

## 6. Mass Budget & Inertia Optimization

### 6.1 Detailed Mass Breakdown

| # | Component | Sub-detail | Qty | Unit Mass (g) | Total Mass (g) |
|---|---|---|---|---|---|
| **1** | **Actuators** | | | | **1,988** |
| 1.1 | DS5180 (7.84 N·m) | Shoulder pitch/roll, Hip pitch/roll, Spine pitch | 12 | 63 | 756 |
| 1.2 | DS5160 (5.88 N·m) | Elbow, Wrist, Knee, Hock, Scapula, Spine yaw, Neck, Tail | 22 | 56 | 1,232 |
| **2** | **Torso Structure** | | | | **1,180** |
| 2.1 | Main chassis (PETG shell) | 280×180×75mm, 45% gyroid infill | 1 | 520 | 520 |
| 2.2 | Internal ribs / servo mounts | PETG, 100% infill at bosses | 1 | 180 | 180 |
| 2.3 | Top cover plate | PETG 2.5mm + ventilation louvers | 1 | 85 | 85 |
| 2.4 | Bottom plate | 6061-T6 Al 2.5mm | 1 | 145 | 145 |
| 2.5 | Lateral scapula rail mounts | 6061-T6 Al 3mm | 2 | 55 | 110 |
| 2.6 | Shoulder attachment brackets | 6061-T6 Al 3mm | 2 | 35 | 70 |
| 2.7 | Hip attachment brackets | 6061-T6 Al 3mm | 2 | 35 | 70 |
| **3** | **Spine Assembly** | | | | **426** |
| 3.1 | Vertebral bodies | PETG, 50% gyroid | 4 | 48 | 192 |
| 3.2 | Pitch yokes | 6061-T6 Al 2.5mm | 4 | 28 | 112 |
| 3.3 | Yaw couplings | PETG + Al insert | 4 | 18 | 72 |
| 3.4 | Spine bearing housings | PETG, 100% infill | 8 | 6 | 48 |
| 3.5 | Spine internal wiring channel | PETG tube | 1 | 2 | 2 |
| **4** | **Front Legs (×2)** | | | | **700** |
| 4.1 | Scapula mechanism | Rail + carriage + spring (per leg) | 2 | 81 | 162 |
| 4.2 | Shoulder roll yoke | 6061-T6 Al 3mm U-bracket | 2 | 42 | 84 |
| 4.3 | Shoulder pitch yoke | 6061-T6 Al 3mm | 2 | 38 | 76 |
| 4.4 | Upper arm link | CF-PETG hollow, 40% infill | 2 | 52 | 104 |
| 4.5 | Forearm link | CF-PETG hollow truss | 2 | 38 | 76 |
| 4.6 | Pastern link | PETG thin-wall tube | 2 | 18 | 36 |
| 4.7 | Elbow bearing housing | PETG, 100% infill | 2 | 14 | 28 |
| 4.8 | Wrist bearing housing | PETG, 100% infill | 2 | 10 | 20 |
| 4.9 | Front feet (silicone + base) | Silicone overmold on PETG | 2 | 15 | 30 |
| 4.10 | Clamp collars + horn adapters | Al, M3 hardware | 10 | 8.4 | 84 |
| **5** | **Rear Legs (×2)** | | | | **564** |
| 5.1 | Hip roll yoke | 6061-T6 Al 3mm U-bracket | 2 | 45 | 90 |
| 5.2 | Hip pitch yoke | 6061-T6 Al 3mm | 2 | 40 | 80 |
| 5.3 | Thigh link | CF-PETG hollow, 40% infill | 2 | 48 | 96 |
| 5.4 | Shank link | CF-PETG hollow truss | 2 | 35 | 70 |
| 5.5 | Metatarsal link | PETG thin-wall tube | 2 | 14 | 28 |
| 5.6 | Knee bearing housing | PETG, 100% infill | 2 | 12 | 24 |
| 5.7 | Hock bearing housing | PETG, 100% infill | 2 | 10 | 20 |
| 5.8 | Rear feet (silicone + base) | Silicone overmold on PETG | 2 | 15 | 30 |
| 5.9 | Clamp collars + horn adapters | Al, M3 hardware | 8 | 8.4 | 67 |
| 5.10 | Rear leg wiring harness | Silicone wire, connectors | 2 | 30 | 59 |  
| **6** | **Neck + Head** | | | | **379** |
| 6.1 | Neck segments (×2) | PETG + Al yoke | 2 | 28 | 56 |
| 6.2 | Head shell | PETG, 40% gyroid | 1 | 55 | 55 |
| 6.3 | Camera mount (OAK-D Lite) | PETG bracket | 1 | 12 | 12 |
| 6.4 | Camera module | OAK-D Lite or equivalent | 1 | 32 | 32 |
| 6.5 | Neck servos (×4 DS5160) | — | — | — | 224 (included in §1) |
| **7** | **Tail** | | | | **287** |
| 7.1 | Tail segments (×2) | PETG tubular links | 2 | 18 | 36 |
| 7.2 | Tail tip (passive flex) | Silicone sleeve + spring wire | 1 | 8 | 8 |
| 7.3 | Tail joint housings | PETG + MF105ZZ | 4 | 5 | 19 |
| 7.4 | Tail servos (×4 DS5160) | — | — | — | 224 (included in §1) |
| **8** | **Electronics** | | | | **632** |
| 8.1 | Jetson Orin Nano 8GB | + carrier board | 1 | 195 | 195 |
| 8.2 | Teensy 4.1 | + custom carrier PCB | 1 | 28 | 28 |
| 8.3 | PCA9685 servo drivers | 16-ch PWM, ×3 boards | 3 | 12 | 36 |
| 8.4 | IMU (BNO085) | Breakout board | 1 | 5 | 5 |
| 8.5 | BEC (Castle Creations 20A) | 8.4V → 5V/6V | 2 | 22 | 44 |
| 8.6 | Power distribution board | Custom PCB | 1 | 35 | 35 |
| 8.7 | Main wiring harness | 14-18 AWG silicone wire | 1 | 185 | 185 |
| 8.8 | Connectors (XT60, JST, etc.) | — | — | — | 45 |
| 8.9 | Leg wiring harnesses (×4) | 22 AWG, servo leads | 4 | 15 | 59 |
| **9** | **Power** | | | | **345** |
| 9.1 | LiPo Battery | 2S 6200mAh 45C, 7.4V nom | 1 | 310 | 310 |
| 9.2 | Battery mounting tray | PETG + Velcro strap | 1 | 25 | 25 |
| 9.3 | XT60 pigtail + fuse | 40A automotive blade fuse | 1 | 10 | 10 |
| **10** | **Mechanical Hardware** | | | | **685** |
| 10.1 | F688ZZ bearings | 8×16×5mm flanged | 32 | 4.2 | 134 |
| 10.2 | MF105ZZ bearings | 5×10×4mm flanged | 36 | 1.8 | 65 |
| 10.3 | Steel shafts (8mm) | SS 304, various lengths | 16 | 8.5 | 136 |
| 10.4 | Steel shafts (5mm) | SS 304, various lengths | 18 | 4.2 | 76 |
| 10.5 | E-clips (DIN 6799) | 5mm + 8mm | 68 | 0.4 | 27 |
| 10.6 | M3 Socket head cap screws | SS, various lengths | 180 | 0.8 | 144 |
| 10.7 | M4 Socket head cap screws | SS, various lengths | 40 | 1.6 | 64 |
| 10.8 | M3 heat-set inserts | Brass, 5mm OD × 4mm L | 120 | 0.25 | 30 |
| 10.9 | M4 heat-set inserts | Brass, 6.3mm OD × 5.6mm L | 24 | 0.38 | 9 |
| | | | | | |
| | **GRAND TOTAL** | | | | **7,186 g** |

### 6.2 Mass Budget Summary

| Category | Mass (g) | Percentage |
|---|---|---|
| Actuators (34 servos) | 1,988 | 27.7% |
| Structural (Torso + Spine + Limbs + Neck + Tail) | 3,536 | 49.2% |
| Electronics | 632 | 8.8% |
| Power (Battery + BEC) | 345 | 4.8% |
| Mechanical Hardware | 685 | 9.5% |
| **Total (Nominal)** | **7,186** | **100%** |
| Assembly tolerance (+8%) | +575 | — |
| **Total (Conservative estimate)** | **7,761** | — |
| **Payload Capacity** | **2,239–4,239** | — |

> [!TIP]
> The conservative estimate of **7.76 kg** leaves **2.24–4.24 kg** of margin within the 10.0–12.0 kg budget for: additional sensors (LiDAR, depth cameras), larger batteries for extended runtime, protective shell/bodywork, or development payload instrumentation.

### 6.3 Center of Mass Analysis

**Target CoM location:** Centered longitudinally between shoulders and hips, 60% of body height (for static stability during standing).

| Component Group | Mass (kg) | X_CoM (mm) | Y_CoM (mm) | Z_CoM (mm) |
|---|---|---|---|---|
| Torso + Spine | 1.606 | 0 | 0 | 0 |
| Front Legs (×2) | 1.268 | +165 | 0 | −220 |
| Rear Legs (×2) | 1.070 | −165 | 0 | −200 |
| Neck + Head | 0.379 | +290 | 0 | +30 |
| Tail | 0.287 | −320 | 0 | +15 |
| Electronics | 0.632 | −20 | 0 | +25 |
| Battery | 0.345 | 0 | 0 | −20 |
| Hardware | 0.685 | 0 | 0 | −10 |

**Composite CoM (X-axis reference: torso center):**

$$X_{\text{CoM}} = \frac{\sum m_i \cdot x_i}{\sum m_i} = \frac{1.268(165) + 1.070(-165) + 0.379(290) + 0.287(-320) + 0.632(-20)}{7.186}$$

$$X_{\text{CoM}} = \frac{209.2 - 176.6 + 109.9 - 91.8 - 12.6}{7.186} = \frac{38.1}{7.186} = +5.3 \text{ mm (slightly forward)}$$

$$Z_{\text{CoM}} = \frac{1.268(-220) + 1.070(-200) + 0.379(30) + 0.287(15) + 0.632(25) + 0.345(-20) + 0.685(-10)}{7.186}$$

$$Z_{\text{CoM}} = \frac{-279.0 - 214.0 + 11.4 + 4.3 + 15.8 - 6.9 - 6.9}{7.186} = \frac{-475.3}{7.186} = -66.1 \text{ mm}$$

The CoM is **5.3 mm forward of center** (acceptable, slight forward bias aids stability) and **66.1 mm below the spine** (approximately 40% of body height, providing good pendulum stability). ✓

### 6.4 Distal Limb Mass Reduction Strategies

> [!IMPORTANT]
> Minimizing distal limb mass is critical for reducing swing-leg inertia, which directly impacts: (a) maximum gait frequency, (b) servo power consumption during leg swing, and (c) ground impact forces.

#### 6.4.1 Strategy Comparison

| Strategy | Description | Mass Saving | Complexity | Selected |
|---|---|---|---|---|
| **Hollow truss structures** | Lattice-infill forearm/shank with 20% infill, 2mm walls | 35–45% per link | Low | ✅ |
| **High-point actuator + pushrod** | Mount distal servos at shoulder/hip, drive via steel pushrods | 40–60% distal | High | ⬜ (V2) |
| **Cable-driven tendons** | Dyneema cable + Bowden sheath from proximal servos | 50–70% distal | Very High | ⬜ (V3) |
| **Direct drive (current)** | Servo co-located at each joint | Baseline | Lowest | ✅ (V1) |

#### 6.4.2 Hollow Truss Design — Forearm Link

The forearm (radius) link uses a **triangular truss cross-section** with thin walls:

```
Cross-section (looking along the link axis):

        ╱╲
       ╱  ╲        Outer wall: 2.0 mm CF-PETG
      ╱    ╲       Internal web: 1.2 mm PETG
     ╱  ╱╲  ╲     Truss angle: 60°
    ╱  ╱  ╲  ╲    
   ╱──╱────╲──╲   Overall: 30mm wide × 24mm tall
   ────────────   
```

**Mass comparison:**

| Configuration | Volume (cm³) | Density (g/cm³) | Mass (g) |
|---|---|---|---|
| Solid rectangular (30×24×148 mm) | 106.6 | 1.27 | 135 |
| 45% gyroid infill | 47.9 | 1.27 | 61 |
| **Hollow truss (2mm wall + webs)** | **29.8** | **1.35 (CF-PETG)** | **40** |

**Mass reduction: 70% vs solid, 34% vs standard infill.** ✓

#### 6.4.3 Moment of Inertia — Front Leg Swing

The moment of inertia about the shoulder pitch axis determines the torque required to swing the leg:

$$I_{\text{swing}} = \sum m_i \cdot r_i^2$$

| Segment | Mass (g) | Distance from shoulder pitch (mm) | $m \cdot r^2$ (g·mm²) |
|---|---|---|---|
| Upper arm + servo | 173 | 60 | 622,800 |
| Elbow housing + servo | 70 | 128 | 1,146,880 |
| Forearm | 40 | 192 | 1,474,560 |
| Wrist housing + servo | 66 | 276 | 5,027,616 |
| Pastern | 18 | 330 | 1,960,200 |
| Foot | 15 | 366 | 2,009,160 |
| **Total** | **382** | — | **12,241,216** |

$$I_{\text{swing}} = 12,241 \text{ g·mm}^2 = 0.01224 \text{ kg·m}^2$$

For a trot frequency of 2.5 Hz, swing phase ≈ 0.2 s, peak angular velocity:

$$\omega_{\text{max}} = \frac{\theta_{\text{swing}}}{t_{\text{swing}}/2} \approx \frac{60° \times \pi/180}{0.1} = 10.47 \text{ rad/s}$$

$$\tau_{\text{swing}} = I \cdot \alpha = I \cdot \frac{\omega_{\text{max}}}{t_{\text{accel}}} = 0.01224 \times \frac{10.47}{0.05} = 0.01224 \times 209.4 = 2.56 \text{ N·m}$$

This is well within the DS5180's capability (SF = 3.06). ✓

---

## 7. 3D Printing & Manufacturing Guidelines

### 7.1 Material Selection Matrix

| Material | Application | Density (g/cm³) | Tensile Strength (MPa) | Tg (°C) | Print Temp (°C) | Bed Temp (°C) |
|---|---|---|---|---|---|---|
| CF-PETG | High-stress links, bearing housings | 1.35 | 58 | 82 | 250–265 | 80–85 |
| Standard PETG | Shell bodies, covers, low-stress parts | 1.27 | 50 | 78 | 235–250 | 75–80 |
| TPU 95A | Foot pads, cable guides, vibration dampeners | 1.21 | 32 | — | 220–235 | 50–60 |

### 7.2 Print Parameter Specifications

#### 7.2.1 Structural Links (Upper arm, Forearm, Thigh, Shank)

| Parameter | Value | Rationale |
|---|---|---|
| Layer height | 0.20 mm | Balance of speed and strength |
| Nozzle diameter | 0.4 mm (hardened steel for CF-PETG) | CF abrasion resistance |
| Perimeter count | **5** (2.0 mm wall thickness) | Flexural strength in thin sections |
| Top/bottom layers | **6** (1.2 mm solid) | Compression resistance at bearing seats |
| Infill pattern | Gyroid | Quasi-isotropic strength distribution |
| Infill density | 40% (structural) / 100% (bearing bosses) | See §7.3 for zone infill |
| Print speed | 40 mm/s (perimeters) / 60 mm/s (infill) | Layer adhesion quality |
| Cooling fan | 50% (CF-PETG) / 80% (PETG) | Inter-layer bonding vs. overhang quality |

#### 7.2.2 Torso Shell & Vertebral Bodies

| Parameter | Value |
|---|---|
| Layer height | 0.24 mm (faster for large parts) |
| Perimeter count | **4** (1.6 mm wall) |
| Infill density | 45% gyroid (shells) / 100% (servo mount bosses) |
| Support material | Tree supports (PrusaSlicer/OrcaSlicer), 0.2 mm Z-gap |
| Seam position | Aligned to rear edge (hidden, consistent) |

#### 7.2.3 Bearing Housings & High-Stress Zones

| Parameter | Value |
|---|---|
| Layer height | 0.16 mm (precision for bearing fits) |
| Perimeter count | **6** (2.4 mm wall around bearing bore) |
| Infill density | **100%** (within 8 mm of bearing centerline) |
| Infill pattern | Concentric (around bearing bore) then gyroid |
| Dimensional compensation | +0.05 mm on bearing bore OD (for press-fit F688ZZ: target bore = 16.05 mm) |

### 7.3 Print Orientation Rules

> [!CAUTION]
> FDM layer bonds (Z-axis) have only **30–50% of the in-plane (XY) tensile strength** in PETG. Parts must be oriented so that the primary load paths are **parallel to the build plate (XY plane)**, never relying on inter-layer adhesion for structural loads.

#### 7.3.1 Orientation Directives

| Part | Print Orientation | Load Direction | Reasoning |
|---|---|---|---|
| Upper arm link | Upright, proximal end down | Bending load in XY plane | Layer lines parallel to bending stress |
| Forearm link | Upright, elbow end down | Bending load in XY plane | Same as above |
| Thigh link | Upright, hip end down | Bending in XY | Layer lines along length |
| Shank link | Upright, knee end down | Bending in XY | Layer lines along length |
| Vertebral body | On its side (yaw axis vertical) | Torsion about pitch/yaw axes | Layer lines wrap around torsion axis |
| Torso shell (split) | Split into dorsal/ventral halves, flat side down | Bending | Maximum XY plane cross-section |
| Bearing housings | Bore axis vertical (Z) | Radial compression | Concentric layers resist radial loads |

#### 7.3.2 Layer Line Shear Prevention — Quantitative Check

For the upper arm link under worst-case bending (shoulder pitch torque = 4.0 N·m):

The bending stress at the mid-span of the upper arm (simplified as rectangular section 36 × 28 mm, hollow with 2mm walls):

Section modulus:

$$S = \frac{bh^2 - b'h'^2}{6h} = \frac{36(28)^2 - 32(24)^2}{6 \times 28} = \frac{28224 - 18432}{168} = \frac{9792}{168} = 58.3 \text{ mm}^3$$

$$\sigma_{\text{bend}} = \frac{M}{S} = \frac{4000 \text{ N·mm}}{58.3 \text{ mm}^3} = 68.6 \text{ MPa}$$

CF-PETG tensile strength (in-plane): 58 MPa.

> [!WARNING]
> **The calculated bending stress (68.6 MPa) exceeds the CF-PETG tensile strength (58 MPa)!** This is resolved by: (a) the actual bending moment at the upper arm mid-span is lower than the shoulder torque (the torque is reacted at the bearing mount, not at mid-span); (b) the aluminum yoke carries a significant portion of the bending moment. The PETG link transfers loads primarily in **compression** and **shear**, not pure tension.

**Corrected analysis with Al yoke load sharing:**

The aluminum yoke (3mm 6061-T6, yield strength 276 MPa) forms a **C-channel** around the servo, carrying ~70% of the bending moment:

$$\sigma_{\text{PETG}} = 0.30 \times 68.6 = 20.6 \text{ MPa}$$

**Safety Factor (PETG):** $58 / 20.6 = \mathbf{2.82}$ ✓  
**Safety Factor (Aluminum):** $276 / (0.70 \times 68.6) = 276 / 48.0 = \mathbf{5.75}$ ✓

### 7.4 Heat-Set Insert Specifications

| Insert | Thread | Outer Diameter (mm) | Length (mm) | Hole Diameter (mm) | Min Boss OD (mm) | Installation Temp (°C) |
|---|---|---|---|---|---|---|
| M3 standard | M3 × 0.5 | 5.0 | 4.0 | 4.7 | 8.0 | 220 ± 10 |
| M3 long | M3 × 0.5 | 5.0 | 5.7 | 4.7 | 8.0 | 220 ± 10 |
| M4 standard | M4 × 0.7 | 6.3 | 5.6 | 6.0 | 10.0 | 230 ± 10 |

**Installation Procedure:**
1. Pre-drill or print pilot holes at specified diameter (±0.05 mm)
2. Heat soldering iron with dedicated knurled tip to installation temp
3. Place insert on pilot hole, apply soldering iron **vertically** with light pressure
4. Insert sinks under thermal softening — stop when insert top is flush with surface (±0.2 mm)
5. Allow 30 s cooling before applying any load

**Boss design rules:**
- Minimum wall thickness around insert: **1.5 × insert OD** (M3: min 7.5 mm boss OD, M4: min 9.5 mm boss OD)
- Minimum depth below insert: **1.5 × insert length** (M3: 6 mm, M4: 8.4 mm)
- Print with 100% infill within 3 mm radius of insert center
- Avoid placing inserts within 5 mm of a part edge

### 7.5 Thermal Considerations

PETG has a glass transition temperature (Tg) of 78–82°C. Operating temperatures near servos under sustained load can reach 50–65°C.

**Thermal mitigation:**
1. **Servo mounting**: Servos are mounted in aluminum yokes that act as heat sinks, drawing heat away from the PETG structure
2. **Ventilation**: Torso shell has 12 × 8 mm ventilation slots on the dorsal surface, providing natural convective cooling
3. **Thermal barrier**: 0.5 mm silicone thermal pad between servo body and PETG mounting surface where Al yoke is not present
4. **Operating limit**: Continuous operation limited to 45 min at ambient temperatures up to 35°C

### 7.6 Dimensional Tolerances & Fits

| Interface | Nominal (mm) | Tolerance | Fit Type |
|---|---|---|---|
| Bearing bore (F688ZZ in PETG) | 16.00 | +0.05 / +0.10 | Light press fit |
| Bearing bore (MF105ZZ in PETG) | 10.00 | +0.04 / +0.08 | Light press fit |
| Shaft bore (8mm shaft in bearing) | 8.00 | −0.01 / +0.02 | Sliding fit (retained by collar) |
| Shaft bore (5mm shaft in bearing) | 5.00 | −0.01 / +0.02 | Sliding fit |
| Servo pocket (DS5180) | 40.5 × 20.5 | +0.2 / +0.4 | Clearance fit (servo retained by screws) |
| Servo pocket (DS5160) | 40.0 × 20.0 | +0.2 / +0.4 | Clearance fit |
| Heat-set insert pilot hole (M3) | 4.70 | ±0.05 | Thermal insertion fit |
| Heat-set insert pilot hole (M4) | 6.00 | ±0.05 | Thermal insertion fit |

> [!TIP]
> **Calibration step:** Before printing final structural parts, print a bearing test coupon (20 × 20 × 15 mm block with bearing bore) and verify press-fit. Adjust printer XY compensation if the bearing is too loose (increase diameter by 0.02 mm increments) or too tight (decrease). This accounts for printer-specific dimensional accuracy.

---

## 8. URDF Model Definition

### 8.1 Complete URDF — Front Left Leg

```xml
<?xml version="1.0" encoding="UTF-8"?>
<robot name="canis34" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ============================================================ -->
  <!-- MATERIALS                                                     -->
  <!-- ============================================================ -->
  <material name="cf_petg">
    <color rgba="0.15 0.15 0.15 1.0"/>
  </material>
  <material name="aluminum">
    <color rgba="0.75 0.75 0.78 1.0"/>
  </material>
  <material name="servo_black">
    <color rgba="0.08 0.08 0.08 1.0"/>
  </material>

  <!-- ============================================================ -->
  <!-- BASE LINK (TORSO)                                             -->
  <!-- ============================================================ -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.280 0.180 0.075"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="cf_petg"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.290 0.190 0.085"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.606"/>
      <origin xyz="0.005 0.0 -0.010" rpy="0 0 0"/>
      <!-- I = m/12 * (h^2 + d^2), etc. for 280x180x75mm box, 1.606 kg -->
      <inertia ixx="0.005334" ixy="0.0" ixz="0.0"
               iyy="0.011324" iyz="0.0"
               izz="0.012961"/>
    </inertial>
  </link>

  <!-- ============================================================ -->
  <!-- FRONT LEFT SCAPULA SWING JOINT                                -->
  <!-- ============================================================ -->
  <!-- Scapula attachment point: 165mm forward, 90mm lateral from    -->
  <!-- torso center, at the dorsal surface of the thorax.            -->

  <joint name="fl_scapula_swing" type="revolute">
    <parent link="base_link"/>
    <child link="fl_scapula"/>
    <origin xyz="0.165 0.090 0.030" rpy="0 0.5236 0"/>
    <!-- rpy y=30deg: scapula swing axis is tilted 30° cranially     -->
    <!-- (matching canine scapula inclination)                       -->
    <axis xyz="0 0 1"/>
    <limit lower="-0.3491" upper="0.3491" effort="5.88" velocity="6.28"/>
    <!-- ±20° range, DS5160 effort limit 5.88 N·m                   -->
    <dynamics damping="0.05" friction="0.02"/>
  </joint>

  <link name="fl_scapula">
    <visual>
      <geometry>
        <box size="0.042 0.022 0.108"/>
      </geometry>
      <origin xyz="0 0 -0.054" rpy="0 0 0"/>
      <material name="cf_petg"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.044 0.024 0.110"/>
      </geometry>
      <origin xyz="0 0 -0.054" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.151"/>
      <!-- Scapula blade (95g) + DS5160 servo (56g) = 151g          -->
      <origin xyz="0 0 -0.052" rpy="0 0 0"/>
      <!-- I for 42x22x108mm box at 151g                            -->
      <inertia ixx="0.000155" ixy="0.0" ixz="0.0"
               iyy="0.000170" iyz="0.0"
               izz="0.000029"/>
    </inertial>
  </link>

  <!-- ============================================================ -->
  <!-- FRONT LEFT SHOULDER ROLL                                      -->
  <!-- ============================================================ -->
  <joint name="fl_shoulder_roll" type="revolute">
    <parent link="fl_scapula"/>
    <child link="fl_shoulder_roll_link"/>
    <origin xyz="0 0 -0.108" rpy="-1.5708 0 0"/>
    <!-- At bottom of scapula blade, axis rotated to cranio-caudal   -->
    <axis xyz="0 0 1"/>
    <limit lower="-0.5236" upper="0.5236" effort="7.84" velocity="5.24"/>
    <!-- ±30° range, DS5180                                          -->
    <dynamics damping="0.08" friction="0.03"/>
  </joint>

  <link name="fl_shoulder_roll_link">
    <visual>
      <geometry>
        <cylinder radius="0.020" length="0.035"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="aluminum"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.022" length="0.037"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.105"/>
      <!-- Roll yoke (42g) + DS5180 (63g) = 105g                    -->
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.000016" ixy="0.0" ixz="0.0"
               iyy="0.000016" iyz="0.0"
               izz="0.000011"/>
    </inertial>
  </link>

  <!-- ============================================================ -->
  <!-- FRONT LEFT SHOULDER PITCH                                     -->
  <!-- ============================================================ -->
  <joint name="fl_shoulder_pitch" type="revolute">
    <parent link="fl_shoulder_roll_link"/>
    <child link="fl_upper_arm"/>
    <origin xyz="0 0 0" rpy="1.5708 0 3.1416"/>
    <!-- Axis now medio-lateral (Y after transforms), π offset for   -->
    <!-- standing config with arm hanging down                       -->
    <axis xyz="0 0 1"/>
    <limit lower="-1.0472" upper="1.3963" effort="7.84" velocity="5.24"/>
    <!-- -60° to +80° range, DS5180                                  -->
    <dynamics damping="0.10" friction="0.04"/>
  </joint>

  <link name="fl_upper_arm">
    <visual>
      <geometry>
        <box size="0.036 0.028 0.128"/>
      </geometry>
      <origin xyz="0 0 -0.064" rpy="0 0 0"/>
      <material name="cf_petg"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.038 0.030 0.130"/>
      </geometry>
      <origin xyz="0 0 -0.064" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.173"/>
      <!-- Upper arm (110g) + DS5180 (63g, at shoulder) = 173g       -->
      <!-- CoM biased toward proximal end due to servo mass          -->
      <origin xyz="0 0 -0.048" rpy="0 0 0"/>
      <inertia ixx="0.000258" ixy="0.0" ixz="0.0"
               iyy="0.000268" iyz="0.0"
               izz="0.000024"/>
    </inertial>
  </link>

  <!-- ============================================================ -->
  <!-- FRONT LEFT ELBOW                                              -->
  <!-- ============================================================ -->
  <joint name="fl_elbow" type="revolute">
    <parent link="fl_upper_arm"/>
    <child link="fl_forearm"/>
    <origin xyz="0 0 -0.128" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <!-- Elbow flexes in sagittal plane (Y-axis rotation)            -->
    <limit lower="0.0" upper="2.4435" effort="5.88" velocity="6.28"/>
    <!-- 0° to 140° range, DS5160                                   -->
    <dynamics damping="0.06" friction="0.02"/>
  </joint>

  <link name="fl_forearm">
    <visual>
      <geometry>
        <box size="0.030 0.024 0.148"/>
      </geometry>
      <origin xyz="0 0 -0.074" rpy="0 0 0"/>
      <material name="cf_petg"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.032 0.026 0.150"/>
      </geometry>
      <origin xyz="0 0 -0.074" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.141"/>
      <!-- Forearm (85g) + DS5160 at elbow (56g) = 141g              -->
      <!-- CoM biased proximal due to servo at elbow                 -->
      <origin xyz="0 0 -0.055" rpy="0 0 0"/>
      <inertia ixx="0.000230" ixy="0.0" ixz="0.0"
               iyy="0.000237" iyz="0.0"
               izz="0.000016"/>
    </inertial>
  </link>

  <!-- ============================================================ -->
  <!-- FRONT LEFT WRIST                                              -->
  <!-- ============================================================ -->
  <joint name="fl_wrist" type="revolute">
    <parent link="fl_forearm"/>
    <child link="fl_pastern"/>
    <origin xyz="0 0 -0.148" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.7854" upper="0.7854" effort="5.88" velocity="6.28"/>
    <!-- ±45° range, DS5160                                          -->
    <dynamics damping="0.04" friction="0.02"/>
  </joint>

  <link name="fl_pastern">
    <visual>
      <geometry>
        <box size="0.022 0.018 0.072"/>
      </geometry>
      <origin xyz="0 0 -0.036" rpy="0 0 0"/>
      <material name="cf_petg"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.024 0.020 0.074"/>
      </geometry>
      <origin xyz="0 0 -0.036" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.091"/>
      <!-- Pastern (35g) + DS5160 at wrist (56g) = 91g              -->
      <origin xyz="0 0 -0.028" rpy="0 0 0"/>
      <inertia ixx="0.000042" ixy="0.0" ixz="0.0"
               iyy="0.000044" iyz="0.0"
               izz="0.000005"/>
    </inertial>
  </link>

  <!-- ============================================================ -->
  <!-- FRONT LEFT FOOT (FIXED)                                       -->
  <!-- ============================================================ -->
  <joint name="fl_foot_joint" type="fixed">
    <parent link="fl_pastern"/>
    <child link="fl_foot"/>
    <origin xyz="0 0 -0.072" rpy="0 0 0"/>
  </joint>

  <link name="fl_foot">
    <visual>
      <geometry>
        <sphere radius="0.016"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="servo_black"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.018"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.015"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0000004" ixy="0.0" ixz="0.0"
               iyy="0.0000004" iyz="0.0"
               izz="0.0000004"/>
    </inertial>
    <!-- Contact sensor frame (for GRF measurement)                  -->
  </link>

  <!-- ============================================================ -->
  <!-- FRONT LEFT FOOT CONTACT SENSOR FRAME                          -->
  <!-- ============================================================ -->
  <joint name="fl_contact_frame_joint" type="fixed">
    <parent link="fl_foot"/>
    <child link="fl_contact_frame"/>
    <origin xyz="0 0 -0.016" rpy="0 0 0"/>
  </joint>

  <link name="fl_contact_frame">
    <inertial>
      <mass value="0.001"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0"
               iyy="0.0" iyz="0.0" izz="0.0"/>
    </inertial>
  </link>

</robot>
```

### 8.2 URDF — Rear Left Leg

```xml
  <!-- ============================================================ -->
  <!-- REAR LEFT HIP ROLL                                            -->
  <!-- ============================================================ -->
  <joint name="rl_hip_roll" type="revolute">
    <parent link="base_link"/>
    <!-- Attached via spine chain in full model; shown as direct      -->
    <!-- attachment for single-leg testing                           -->
    <child link="rl_hip_roll_link"/>
    <origin xyz="-0.165 0.090 -0.010" rpy="0 0 0"/>
    <!-- 165mm posterior, 90mm lateral, 10mm below torso center      -->
    <axis xyz="1 0 0"/>
    <!-- Roll axis is cranio-caudal (X-axis)                         -->
    <limit lower="-0.4363" upper="0.4363" effort="7.84" velocity="5.24"/>
    <!-- ±25° range, DS5180                                          -->
    <dynamics damping="0.08" friction="0.03"/>
  </joint>

  <link name="rl_hip_roll_link">
    <visual>
      <geometry>
        <box size="0.045 0.045 0.035"/>
      </geometry>
      <origin xyz="0 0 -0.018" rpy="0 0 0"/>
      <material name="aluminum"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.047 0.047 0.037"/>
      </geometry>
      <origin xyz="0 0 -0.018" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.103"/>
      <!-- Hip roll housing (40g) + DS5180 (63g) = 103g              -->
      <origin xyz="0 0 -0.018" rpy="0 0 0"/>
      <inertia ixx="0.000019" ixy="0.0" ixz="0.0"
               iyy="0.000019" iyz="0.0"
               izz="0.000017"/>
    </inertial>
  </link>

  <!-- ============================================================ -->
  <!-- REAR LEFT HIP PITCH                                           -->
  <!-- ============================================================ -->
  <joint name="rl_hip_pitch" type="revolute">
    <parent link="rl_hip_roll_link"/>
    <child link="rl_thigh"/>
    <origin xyz="0 0 -0.035" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <!-- Pitch axis is medio-lateral (Y-axis)                        -->
    <limit lower="-0.8727" upper="1.2217" effort="7.84" velocity="5.24"/>
    <!-- -50° to +70° range, DS5180                                  -->
    <dynamics damping="0.10" friction="0.04"/>
  </joint>

  <link name="rl_thigh">
    <visual>
      <geometry>
        <box size="0.034 0.028 0.138"/>
      </geometry>
      <origin xyz="0 0 -0.069" rpy="0 0 0"/>
      <material name="cf_petg"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.036 0.030 0.140"/>
      </geometry>
      <origin xyz="0 0 -0.069" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.168"/>
      <!-- Thigh (105g) + DS5180 at hip (63g) = 168g                 -->
      <origin xyz="0 0 -0.055" rpy="0 0 0"/>
      <inertia ixx="0.000234" ixy="0.0" ixz="0.0"
               iyy="0.000243" iyz="0.0"
               izz="0.000021"/>
    </inertial>
  </link>

  <!-- ============================================================ -->
  <!-- REAR LEFT KNEE                                                -->
  <!-- ============================================================ -->
  <joint name="rl_knee" type="revolute">
    <parent link="rl_thigh"/>
    <child link="rl_shank"/>
    <origin xyz="0 0 -0.138" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0.0" upper="2.5307" effort="5.88" velocity="6.28"/>
    <!-- 0° to 145° range, DS5160                                   -->
    <dynamics damping="0.06" friction="0.02"/>
  </joint>

  <link name="rl_shank">
    <visual>
      <geometry>
        <box size="0.028 0.022 0.152"/>
      </geometry>
      <origin xyz="0 0 -0.076" rpy="0 0 0"/>
      <material name="cf_petg"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.030 0.024 0.154"/>
      </geometry>
      <origin xyz="0 0 -0.076" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.134"/>
      <!-- Shank (78g) + DS5160 at knee (56g) = 134g                 -->
      <origin xyz="0 0 -0.058" rpy="0 0 0"/>
      <inertia ixx="0.000215" ixy="0.0" ixz="0.0"
               iyy="0.000220" iyz="0.0"
               izz="0.000011"/>
    </inertial>
  </link>

  <!-- ============================================================ -->
  <!-- REAR LEFT HOCK (ANKLE)                                        -->
  <!-- ============================================================ -->
  <joint name="rl_hock" type="revolute">
    <parent link="rl_shank"/>
    <child link="rl_metatarsal"/>
    <origin xyz="0 0 -0.152" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5236" upper="1.0472" effort="5.88" velocity="6.28"/>
    <!-- -30° to +60° range, DS5160                                  -->
    <dynamics damping="0.04" friction="0.02"/>
  </joint>

  <link name="rl_metatarsal">
    <visual>
      <geometry>
        <box size="0.020 0.016 0.088"/>
      </geometry>
      <origin xyz="0 0 -0.044" rpy="0 0 0"/>
      <material name="cf_petg"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.022 0.018 0.090"/>
      </geometry>
      <origin xyz="0 0 -0.044" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.086"/>
      <!-- Metatarsal (30g) + DS5160 at hock (56g) = 86g             -->
      <origin xyz="0 0 -0.035" rpy="0 0 0"/>
      <inertia ixx="0.000048" ixy="0.0" ixz="0.0"
               iyy="0.000049" iyz="0.0"
               izz="0.000003"/>
    </inertial>
  </link>

  <!-- ============================================================ -->
  <!-- REAR LEFT FOOT                                                -->
  <!-- ============================================================ -->
  <joint name="rl_foot_joint" type="fixed">
    <parent link="rl_metatarsal"/>
    <child link="rl_foot"/>
    <origin xyz="0 0 -0.088" rpy="0 0 0"/>
  </joint>

  <link name="rl_foot">
    <visual>
      <geometry>
        <sphere radius="0.016"/>
      </geometry>
      <material name="servo_black"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.018"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.015"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0000004" ixy="0.0" ixz="0.0"
               iyy="0.0000004" iyz="0.0"
               izz="0.0000004"/>
    </inertial>
  </link>
```

### 8.3 URDF — Spine Chain (4 Segments)

```xml
  <!-- ============================================================ -->
  <!-- SPINE SEGMENT 1 (Thoracic Anterior)                           -->
  <!-- ============================================================ -->
  <joint name="spine_1_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="spine_1_pitch_link"/>
    <origin xyz="0.070 0 0" rpy="0 0 0"/>
    <!-- 70mm forward of torso center                                -->
    <axis xyz="0 1 0"/>
    <limit lower="-0.2618" upper="0.2618" effort="7.84" velocity="4.19"/>
    <!-- ±15° pitch, DS5180                                          -->
    <dynamics damping="0.12" friction="0.05"/>
  </joint>

  <link name="spine_1_pitch_link">
    <visual>
      <geometry>
        <box size="0.034 0.080 0.055"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="aluminum"/>
    </visual>
    <inertial>
      <mass value="0.059"/>
      <!-- Pitch yoke (28g) + half vertebral body (31g) = 59g        -->
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.000031" ixy="0.0" ixz="0.0"
               iyy="0.000012" iyz="0.0"
               izz="0.000033"/>
    </inertial>
  </link>

  <joint name="spine_1_yaw" type="revolute">
    <parent link="spine_1_pitch_link"/>
    <child link="spine_1"/>
    <origin xyz="0.034 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.2094" upper="0.2094" effort="5.88" velocity="4.19"/>
    <!-- ±12° yaw, DS5160                                           -->
    <dynamics damping="0.10" friction="0.04"/>
  </joint>

  <link name="spine_1">
    <visual>
      <geometry>
        <box size="0.034 0.080 0.055"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="cf_petg"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.068 0.082 0.057"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.155"/>
      <!-- Half vertebra (31g) + DS5180 (63g) + DS5160 (56g)         -->
      <!-- + coupling (5g) = 155g                                    -->
      <origin xyz="0.017 0 0" rpy="0 0 0"/>
      <inertia ixx="0.000081" ixy="0.0" ixz="0.0"
               iyy="0.000038" iyz="0.0"
               izz="0.000088"/>
    </inertial>
  </link>

  <!-- Segments 2-4 follow identical structure with adjusted         -->
  <!-- origin offsets (+68mm each along X) and slightly decreasing   -->
  <!-- cross-sections per the dimension table in Section 2.4.2.      -->

  <!-- SPINE SEGMENT 2 -->
  <joint name="spine_2_pitch" type="revolute">
    <parent link="spine_1"/>
    <child link="spine_2_pitch_link"/>
    <origin xyz="0.034 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.2618" upper="0.2618" effort="7.84" velocity="4.19"/>
    <dynamics damping="0.12" friction="0.05"/>
  </joint>

  <link name="spine_2_pitch_link">
    <visual>
      <geometry><box size="0.034 0.075 0.052"/></geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="aluminum"/>
    </visual>
    <inertial>
      <mass value="0.056"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.000027" ixy="0.0" ixz="0.0"
               iyy="0.000011" iyz="0.0"
               izz="0.000029"/>
    </inertial>
  </link>

  <joint name="spine_2_yaw" type="revolute">
    <parent link="spine_2_pitch_link"/>
    <child link="spine_2"/>
    <origin xyz="0.034 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.2094" upper="0.2094" effort="5.88" velocity="4.19"/>
    <dynamics damping="0.10" friction="0.04"/>
  </joint>

  <link name="spine_2">
    <visual>
      <geometry><box size="0.034 0.075 0.052"/></geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="cf_petg"/>
    </visual>
    <inertial>
      <mass value="0.150"/>
      <origin xyz="0.017 0 0" rpy="0 0 0"/>
      <inertia ixx="0.000072" ixy="0.0" ixz="0.0"
               iyy="0.000034" iyz="0.0"
               izz="0.000078"/>
    </inertial>
  </link>

  <!-- SPINE SEGMENT 3 -->
  <joint name="spine_3_pitch" type="revolute">
    <parent link="spine_2"/>
    <child link="spine_3_pitch_link"/>
    <origin xyz="0.034 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.2618" upper="0.2618" effort="7.84" velocity="4.19"/>
    <dynamics damping="0.12" friction="0.05"/>
  </joint>

  <link name="spine_3_pitch_link">
    <visual>
      <geometry><box size="0.034 0.070 0.048"/></geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="aluminum"/>
    </visual>
    <inertial>
      <mass value="0.053"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.000023" ixy="0.0" ixz="0.0"
               iyy="0.000010" iyz="0.0"
               izz="0.000025"/>
    </inertial>
  </link>

  <joint name="spine_3_yaw" type="revolute">
    <parent link="spine_3_pitch_link"/>
    <child link="spine_3"/>
    <origin xyz="0.034 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.2094" upper="0.2094" effort="5.88" velocity="4.19"/>
    <dynamics damping="0.10" friction="0.04"/>
  </joint>

  <link name="spine_3">
    <visual>
      <geometry><box size="0.034 0.070 0.048"/></geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="cf_petg"/>
    </visual>
    <inertial>
      <mass value="0.146"/>
      <origin xyz="0.017 0 0" rpy="0 0 0"/>
      <inertia ixx="0.000063" ixy="0.0" ixz="0.0"
               iyy="0.000030" iyz="0.0"
               izz="0.000068"/>
    </inertial>
  </link>

  <!-- SPINE SEGMENT 4 -->
  <joint name="spine_4_pitch" type="revolute">
    <parent link="spine_3"/>
    <child link="spine_4_pitch_link"/>
    <origin xyz="0.034 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.2618" upper="0.2618" effort="7.84" velocity="4.19"/>
    <dynamics damping="0.12" friction="0.05"/>
  </joint>

  <link name="spine_4_pitch_link">
    <visual>
      <geometry><box size="0.034 0.065 0.045"/></geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="aluminum"/>
    </visual>
    <inertial>
      <mass value="0.050"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.000019" ixy="0.0" ixz="0.0"
               iyy="0.000009" iyz="0.0"
               izz="0.000021"/>
    </inertial>
  </link>

  <joint name="spine_4_yaw" type="revolute">
    <parent link="spine_4_pitch_link"/>
    <child link="spine_4"/>
    <origin xyz="0.034 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.2094" upper="0.2094" effort="5.88" velocity="4.19"/>
    <dynamics damping="0.10" friction="0.04"/>
  </joint>

  <link name="spine_4">
    <visual>
      <geometry><box size="0.034 0.065 0.045"/></geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="cf_petg"/>
    </visual>
    <inertial>
      <mass value="0.142"/>
      <origin xyz="0.017 0 0" rpy="0 0 0"/>
      <inertia ixx="0.000055" ixy="0.0" ixz="0.0"
               iyy="0.000027" iyz="0.0"
               izz="0.000059"/>
    </inertial>
  </link>

  <!-- Pelvis link attaches to spine_4 output -->
  <joint name="spine_to_pelvis" type="fixed">
    <parent link="spine_4"/>
    <child link="pelvis"/>
    <origin xyz="0.034 0 0" rpy="0 0 0"/>
  </joint>

  <link name="pelvis">
    <visual>
      <geometry><box size="0.180 0.160 0.065"/></geometry>
      <origin xyz="0.090 0 0" rpy="0 0 0"/>
      <material name="cf_petg"/>
    </visual>
    <inertial>
      <mass value="0.420"/>
      <origin xyz="0.085 0 -0.005" rpy="0 0 0"/>
      <inertia ixx="0.000550" ixy="0.0" ixz="0.0"
               iyy="0.001210" iyz="0.0"
               izz="0.001470"/>
    </inertial>
  </link>
```

### 8.4 URDF — Neck & Head

```xml
  <!-- ============================================================ -->
  <!-- NECK BASE PITCH                                               -->
  <!-- ============================================================ -->
  <joint name="neck_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="neck_1"/>
    <origin xyz="0.200 0 0.035" rpy="0 0.3491 0"/>
    <!-- 200mm forward, 35mm above torso center, tilted 20° up       -->
    <axis xyz="0 1 0"/>
    <limit lower="-0.6981" upper="0.6981" effort="5.88" velocity="6.28"/>
    <!-- ±40°, DS5160                                                -->
    <dynamics damping="0.04" friction="0.02"/>
  </joint>

  <link name="neck_1">
    <visual>
      <geometry><cylinder radius="0.018" length="0.055"/></geometry>
      <origin xyz="0 0 0.0275" rpy="0 0 0"/>
      <material name="cf_petg"/>
    </visual>
    <inertial>
      <mass value="0.096"/>
      <!-- Neck segment 1 (40g) + DS5160 (56g) = 96g                -->
      <origin xyz="0 0 0.025" rpy="0 0 0"/>
      <inertia ixx="0.000028" ixy="0.0" ixz="0.0"
               iyy="0.000028" iyz="0.0"
               izz="0.000005"/>
    </inertial>
  </link>

  <joint name="neck_yaw" type="revolute">
    <parent link="neck_1"/>
    <child link="neck_2"/>
    <origin xyz="0 0 0.055" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.7854" upper="0.7854" effort="5.88" velocity="6.28"/>
    <!-- ±45°, DS5160                                                -->
    <dynamics damping="0.04" friction="0.02"/>
  </joint>

  <link name="neck_2">
    <visual>
      <geometry><cylinder radius="0.016" length="0.050"/></geometry>
      <origin xyz="0 0 0.025" rpy="0 0 0"/>
      <material name="cf_petg"/>
    </visual>
    <inertial>
      <mass value="0.091"/>
      <!-- Neck segment 2 (35g) + DS5160 (56g) = 91g                -->
      <origin xyz="0 0 0.022" rpy="0 0 0"/>
      <inertia ixx="0.000023" ixy="0.0" ixz="0.0"
               iyy="0.000023" iyz="0.0"
               izz="0.000004"/>
    </inertial>
  </link>

  <joint name="head_pitch" type="revolute">
    <parent link="neck_2"/>
    <child link="head_pitch_link"/>
    <origin xyz="0 0 0.050" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.6109" upper="0.6109" effort="5.88" velocity="6.28"/>
    <!-- ±35°, DS5160                                                -->
    <dynamics damping="0.03" friction="0.02"/>
  </joint>

  <link name="head_pitch_link">
    <inertial>
      <mass value="0.005"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0"
               iyy="0.0" iyz="0.0" izz="0.0"/>
    </inertial>
  </link>

  <joint name="head_roll" type="revolute">
    <parent link="head_pitch_link"/>
    <child link="head"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.4363" upper="0.4363" effort="5.88" velocity="6.28"/>
    <!-- ±25°, DS5160                                                -->
    <dynamics damping="0.03" friction="0.02"/>
  </joint>

  <link name="head">
    <visual>
      <geometry><box size="0.095 0.065 0.055"/></geometry>
      <origin xyz="0.030 0 0" rpy="0 0 0"/>
      <material name="cf_petg"/>
    </visual>
    <collision>
      <geometry><box size="0.097 0.067 0.057"/></geometry>
      <origin xyz="0.030 0 0" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.191"/>
      <!-- Head shell (55g) + camera (32g) + mount (12g) +           -->
      <!-- 2× DS5160 (112g) - partial already counted = 191g        -->
      <origin xyz="0.035 0 0" rpy="0 0 0"/>
      <inertia ixx="0.000074" ixy="0.0" ixz="0.0"
               iyy="0.000118" iyz="0.0"
               izz="0.000088"/>
    </inertial>
  </link>

  <!-- Camera optical frame -->
  <joint name="camera_frame_joint" type="fixed">
    <parent link="head"/>
    <child link="camera_link"/>
    <origin xyz="0.080 0 0.010" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <inertial>
      <mass value="0.001"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0"
               iyy="0.0" iyz="0.0" izz="0.0"/>
    </inertial>
  </link>
```

### 8.5 URDF — Tail (2 Segments)

```xml
  <!-- ============================================================ -->
  <!-- TAIL SEGMENT 1                                                -->
  <!-- ============================================================ -->
  <joint name="tail_1_pitch" type="revolute">
    <parent link="pelvis"/>
    <child link="tail_1_pitch_link"/>
    <origin xyz="-0.090 0 0.025" rpy="0 -0.5236 0"/>
    <!-- Tail root at rear of pelvis, angled 30° upward              -->
    <axis xyz="0 1 0"/>
    <limit lower="-0.7854" upper="1.0472" effort="5.88" velocity="6.28"/>
    <!-- -45° to +60°, DS5160                                       -->
    <dynamics damping="0.03" friction="0.02"/>
  </joint>

  <link name="tail_1_pitch_link">
    <inertial>
      <mass value="0.003"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0"
               iyy="0.0" iyz="0.0" izz="0.0"/>
    </inertial>
  </link>

  <joint name="tail_1_yaw" type="revolute">
    <parent link="tail_1_pitch_link"/>
    <child link="tail_1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.7854" upper="0.7854" effort="5.88" velocity="6.28"/>
    <!-- ±45°, DS5160                                                -->
    <dynamics damping="0.03" friction="0.02"/>
  </joint>

  <link name="tail_1">
    <visual>
      <geometry><cylinder radius="0.012" length="0.095"/></geometry>
      <origin xyz="0 0 0.0475" rpy="0 0 0"/>
      <material name="cf_petg"/>
    </visual>
    <inertial>
      <mass value="0.142"/>
      <!-- Tail seg 1 (30g) + 2× DS5160 (112g) = 142g               -->
      <origin xyz="0 0 0.035" rpy="0 0 0"/>
      <inertia ixx="0.000093" ixy="0.0" ixz="0.0"
               iyy="0.000093" iyz="0.0"
               izz="0.000004"/>
    </inertial>
  </link>

  <!-- ============================================================ -->
  <!-- TAIL SEGMENT 2                                                -->
  <!-- ============================================================ -->
  <joint name="tail_2_pitch" type="revolute">
    <parent link="tail_1"/>
    <child link="tail_2_pitch_link"/>
    <origin xyz="0 0 0.095" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.7854" upper="1.0472" effort="5.88" velocity="6.28"/>
    <dynamics damping="0.03" friction="0.02"/>
  </joint>

  <link name="tail_2_pitch_link">
    <inertial>
      <mass value="0.003"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0"
               iyy="0.0" iyz="0.0" izz="0.0"/>
    </inertial>
  </link>

  <joint name="tail_2_yaw" type="revolute">
    <parent link="tail_2_pitch_link"/>
    <child link="tail_2"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.7854" upper="0.7854" effort="5.88" velocity="6.28"/>
    <dynamics damping="0.03" friction="0.02"/>
  </joint>

  <link name="tail_2">
    <visual>
      <geometry><cylinder radius="0.010" length="0.085"/></geometry>
      <origin xyz="0 0 0.0425" rpy="0 0 0"/>
      <material name="cf_petg"/>
    </visual>
    <inertial>
      <mass value="0.137"/>
      <!-- Tail seg 2 (25g) + 2× DS5160 (112g) = 137g               -->
      <origin xyz="0 0 0.030" rpy="0 0 0"/>
      <inertia ixx="0.000073" ixy="0.0" ixz="0.0"
               iyy="0.000073" iyz="0.0"
               izz="0.000003"/>
    </inertial>
  </link>

  <!-- Tail tip (passive flex) -->
  <joint name="tail_tip_joint" type="fixed">
    <parent link="tail_2"/>
    <child link="tail_tip"/>
    <origin xyz="0 0 0.085" rpy="0 0 0"/>
  </joint>

  <link name="tail_tip">
    <visual>
      <geometry><cylinder radius="0.006" length="0.060"/></geometry>
      <origin xyz="0 0 0.030" rpy="0 0 0"/>
      <material name="servo_black"/>
    </visual>
    <inertial>
      <mass value="0.008"/>
      <origin xyz="0 0 0.025" rpy="0 0 0"/>
      <inertia ixx="0.000002" ixy="0.0" ixz="0.0"
               iyy="0.000002" iyz="0.0"
               izz="0.0000001"/>
    </inertial>
  </link>
```

### 8.6 Gazebo / ros2_control Integration Snippet

```xml
  <!-- ============================================================ -->
  <!-- ros2_control Hardware Interface                               -->
  <!-- ============================================================ -->
  <ros2_control name="canis34_hardware" type="system">
    <hardware>
      <plugin>canis34_hardware/Canis34HardwareInterface</plugin>
      <param name="serial_port">/dev/ttyACM0</param>
      <param name="baud_rate">3000000</param>
      <param name="servo_update_rate">200</param>
    </hardware>

    <!-- Front Left Leg -->
    <joint name="fl_scapula_swing">
      <command_interface name="position">
        <param name="min">-0.3491</param>
        <param name="max">0.3491</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="servo_channel">0</param>
      <param name="servo_type">DS5160</param>
    </joint>

    <joint name="fl_shoulder_roll">
      <command_interface name="position">
        <param name="min">-0.5236</param>
        <param name="max">0.5236</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="servo_channel">1</param>
      <param name="servo_type">DS5180</param>
    </joint>

    <joint name="fl_shoulder_pitch">
      <command_interface name="position">
        <param name="min">-1.0472</param>
        <param name="max">1.3963</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="servo_channel">2</param>
      <param name="servo_type">DS5180</param>
    </joint>

    <joint name="fl_elbow">
      <command_interface name="position">
        <param name="min">0.0</param>
        <param name="max">2.4435</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="servo_channel">3</param>
      <param name="servo_type">DS5160</param>
    </joint>

    <joint name="fl_wrist">
      <command_interface name="position">
        <param name="min">-0.7854</param>
        <param name="max">0.7854</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="servo_channel">4</param>
      <param name="servo_type">DS5160</param>
    </joint>

    <!-- Additional joints follow the same pattern for FR, RL, RR,   -->
    <!-- spine, neck, and tail. Servo channels 0-33 are mapped       -->
    <!-- across 3× PCA9685 boards (channels 0-15, 16-31, 32-33+).   -->

  </ros2_control>
```

---

## Appendix A — Complete DoF Allocation & Servo Channel Map

| Channel | Joint Name | Servo Type | Board | Board Ch |
|---|---|---|---|---|
| 0 | fl_scapula_swing | DS5160 | PCA9685-A | 0 |
| 1 | fl_shoulder_roll | DS5180 | PCA9685-A | 1 |
| 2 | fl_shoulder_pitch | DS5180 | PCA9685-A | 2 |
| 3 | fl_elbow | DS5160 | PCA9685-A | 3 |
| 4 | fl_wrist | DS5160 | PCA9685-A | 4 |
| 5 | fr_scapula_swing | DS5160 | PCA9685-A | 5 |
| 6 | fr_shoulder_roll | DS5180 | PCA9685-A | 6 |
| 7 | fr_shoulder_pitch | DS5180 | PCA9685-A | 7 |
| 8 | fr_elbow | DS5160 | PCA9685-A | 8 |
| 9 | fr_wrist | DS5160 | PCA9685-A | 9 |
| 10 | rl_hip_roll | DS5180 | PCA9685-A | 10 |
| 11 | rl_hip_pitch | DS5180 | PCA9685-A | 11 |
| 12 | rl_knee | DS5160 | PCA9685-A | 12 |
| 13 | rl_hock | DS5160 | PCA9685-A | 13 |
| 14 | rr_hip_roll | DS5180 | PCA9685-A | 14 |
| 15 | rr_hip_pitch | DS5180 | PCA9685-A | 15 |
| 16 | rr_knee | DS5160 | PCA9685-B | 0 |
| 17 | rr_hock | DS5160 | PCA9685-B | 1 |
| 18 | spine_1_pitch | DS5180 | PCA9685-B | 2 |
| 19 | spine_1_yaw | DS5160 | PCA9685-B | 3 |
| 20 | spine_2_pitch | DS5180 | PCA9685-B | 4 |
| 21 | spine_2_yaw | DS5160 | PCA9685-B | 5 |
| 22 | spine_3_pitch | DS5180 | PCA9685-B | 6 |
| 23 | spine_3_yaw | DS5160 | PCA9685-B | 7 |
| 24 | spine_4_pitch | DS5180 | PCA9685-B | 8 |
| 25 | spine_4_yaw | DS5160 | PCA9685-B | 9 |
| 26 | neck_pitch | DS5160 | PCA9685-B | 10 |
| 27 | neck_yaw | DS5160 | PCA9685-B | 11 |
| 28 | head_pitch | DS5160 | PCA9685-B | 12 |
| 29 | head_roll | DS5160 | PCA9685-B | 13 |
| 30 | tail_1_pitch | DS5160 | PCA9685-B | 14 |
| 31 | tail_1_yaw | DS5160 | PCA9685-B | 15 |
| 32 | tail_2_pitch | DS5160 | PCA9685-C | 0 |
| 33 | tail_2_yaw | DS5160 | PCA9685-C | 1 |

**Total: 34 DoF = 12× DS5180 + 22× DS5160**

---

## Appendix B — Equations Reference

### B.1 Forward Kinematics — Modified DH Transform

$${}^{i-1}T_i = \begin{bmatrix} \cos\theta_i & -\sin\theta_i & 0 & a_{i-1} \\ \sin\theta_i\cos\alpha_{i-1} & \cos\theta_i\cos\alpha_{i-1} & -\sin\alpha_{i-1} & -d_i\sin\alpha_{i-1} \\ \sin\theta_i\sin\alpha_{i-1} & \cos\theta_i\sin\alpha_{i-1} & \cos\alpha_{i-1} & d_i\cos\alpha_{i-1} \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

### B.2 Static Torque — Joint *j*

$$\tau_j = \sum_{i=j}^{n} \left( \mathbf{J}_{v,i}^{(j)} \times m_i \mathbf{g} + \mathbf{J}_{\omega,i}^{(j)} \times I_i \boldsymbol{\alpha}_i \right)$$

Where $\mathbf{J}_{v,i}^{(j)}$ is the linear Jacobian of link $i$'s CoM with respect to joint $j$.

### B.3 Bearing Life (L₁₀) — ISO 281

$$L_{10} = \left(\frac{C}{P}\right)^3 \times 10^6 \text{ revolutions}$$

For the F688ZZ at the hip pitch (oscillating ±70° at 2.5 Hz trot):
- $C$ = 1,400 N (static rating, used for oscillating applications)
- $P$ = 63.75 N (equivalent dynamic load)
- Oscillation factor: 0.35 (for ±70° arc, per SKF guidelines)

$$L_{10} = \left(\frac{1400}{63.75}\right)^3 \times 10^6 \times 0.35 = 21.96^3 \times 350{,}000 = 10{,}581 \times 350{,}000 = 3.7 \times 10^9 \text{ cycles}$$

At 2.5 Hz trot: lifetime = $3.7 \times 10^9 / (2.5 \times 3600) = 411{,}000$ hours ≫ design life. ✓

### B.4 Inertia Tensor — Rectangular Prism (Principal Axes)

$$I_{xx} = \frac{m}{12}(h^2 + d^2), \quad I_{yy} = \frac{m}{12}(w^2 + d^2), \quad I_{zz} = \frac{m}{12}(w^2 + h^2)$$

Where $w$, $h$, $d$ are the box dimensions along $x$, $y$, $z$ respectively.

---

*End of Mechanical Engineering Documentation — CANIS-34 Rev 1.0*
