# SYSTEM ARCHITECTURE
## UAV Interception System (Academic Prototype)

---

## 1. Overview

This document defines the high-level architecture of the UAV Interception System. It describes the functional blocks, data flow, interfaces, and responsibilities of each subsystem. It serves as the primary reference for all simulation, estimation, guidance, and control work.

---

## 2. High-Level Pipeline
[ Target UAV ]
|
v
[ Camera / Sensor ]
|
v
[ Detection ] <-- YOLO / OpenCV
|
v
[ Tracking ] <-- Kalman Filter / Optical Flow
|
v
[ State Estimation ] <-- Position, Velocity, Acceleration
|
v
[ Trajectory Prediction ] <-- Constant Velocity / Constant Acceleration
|
v
[ Intercept Point Calculation ]
|
v
[ Guidance Law ] <-- Proportional Navigation
|
v
[ Flight Controller ] <-- PID / LQR
|
v
[ Aircraft Dynamics ] <-- 6-DOF Model
|
v
[ Feedback ] <-- Sensors, Estimation

---

## 3. Functional Blocks

### 3.1 Detection Block

| Item | Description |
|---|---|
| **Input** | Camera frame (RGB or Thermal) |
| **Output** | Bounding box (x, y, w, h), confidence score, class label |
| **Algorithm** | YOLOv8 (or equivalent) |
| **Sampling Rate** | 30 FPS |
| **Latency Target** | < 50 ms |
| **Software** | Python, OpenCV, PyTorch |
| **Responsible** | Perception Team |
| **Uncertainty** | False positives, missed detections in low contrast |

---

### 3.2 Tracking Block

| Item | Description |
|---|---|
| **Input** | Bounding boxes from Detection |
| **Output** | Smoothed target position in image frame |
| **Algorithm** | Kalman Filter / SORT / DeepSORT |
| **Sampling Rate** | 30 FPS |
| **Latency Target** | < 30 ms |
| **Software** | Python, FilterPy |
| **Responsible** | Perception Team |
| **Uncertainty** | Occlusion, sudden maneuvers |

---

### 3.3 State Estimation Block

| Item | Description |
|---|---|
| **Input** | Target position (image + GPS if available) |
| **Output** | Estimated position, velocity, acceleration in NED frame |
| **Algorithm** | Extended Kalman Filter (EKF) |
| **Sampling Rate** | 50 Hz |
| **Latency Target** | < 20 ms |
| **Software** | Python, FilterPy / MATLAB |
| **Responsible** | Estimation Team |
| **Uncertainty** | Sensor noise, GPS error, camera calibration |

---

### 3.4 Trajectory Prediction Block

| Item | Description |
|---|---|
| **Input** | Estimated state (position, velocity, acceleration) |
| **Output** | Predicted target position at time t + Δt |
| **Algorithm** | Constant Velocity / Constant Acceleration |
| **Prediction Horizon** | 0.5 – 2.0 seconds |
| **Sampling Rate** | 50 Hz |
| **Software** | Python, NumPy |
| **Responsible** | Guidance Team |
| **Uncertainty** | Target maneuver, model mismatch |

---

### 3.5 Intercept Point Calculation Block

| Item | Description |
|---|---|
| **Input** | Predicted target position, interceptor state |
| **Output** | Aim point (3D coordinates) |
| **Algorithm** | Iterative solution of intercept equation |
| **Sampling Rate** | 50 Hz |
| **Software** | Python, NumPy |
| **Responsible** | Guidance Team |
| **Uncertainty** | Numerical convergence |

---

### 3.6 Guidance Law Block

| Item | Description |
|---|---|
| **Input** | Aim point, interceptor state |
| **Output** | Desired acceleration / attitude commands |
| **Algorithm** | Proportional Navigation (PN) |
| **Sampling Rate** | 50 Hz |
| **Software** | Python / MATLAB |
| **Responsible** | Guidance Team |
| **Uncertainty** | Saturation, noise amplification |

---

### 3.7 Flight Controller Block

| Item | Description |
|---|---|
| **Input** | Desired attitude / trajectory |
| **Output** | Control surface commands (aileron, elevator, rudder, throttle) |
| **Algorithm** | PID / LQR |
| **Sampling Rate** | 100 Hz |
| **Software** | MATLAB/Simulink, ArduPilot |
| **Responsible** | Control Team |
| **Uncertainty** | Actuator delay, saturation |

---

### 3.8 Aircraft Dynamics Block

| Item | Description |
|---|---|
| **Input** | Control surface commands, thrust |
| **Output** | New state (position, velocity, attitude) |
| **Model** | 6-DOF (12 states) |
| **Integration Rate** | 200 Hz |
| **Software** | MATLAB/Simulink |
| **Responsible** | Modeling Team |
| **Uncertainty** | Aerodynamic coefficients, mass properties |

---

## 4. Data Flow Summary

| Source | Destination | Data Type | Rate |
|---|---|---|---|
| Camera | Detection | Image frame | 30 FPS |
| Detection | Tracking | Bounding box | 30 FPS |
| Tracking | State Estimation | Target position (image) | 30 FPS |
| State Estimation | Prediction | State vector | 50 Hz |
| Prediction | Intercept Point | Predicted position | 50 Hz |
| Intercept Point | Guidance | Aim point | 50 Hz |
| Guidance | Flight Controller | Desired attitude | 50 Hz |
| Flight Controller | Aircraft Dynamics | Control commands | 100 Hz |
| Aircraft Dynamics | Feedback | State vector | 200 Hz |

---

## 5. Coordinate Frames

| Frame | Description | Usage |
|---|---|---|
| **NED** | North-East-Down | Global navigation |
| **Body** | Fixed to aircraft | Forces, moments |
| **Camera** | Fixed to camera | Detection, tracking |
| **Target** | Fixed to target | Relative motion |

Transformations documented in `docs/architecture/frames.md`.

---

## 6. Requirements (High-Level)

| ID | Requirement | Priority |
|---|---|---|
| R1 | Detect small UAV at range 1–2 km | High |
| R2 | Track target continuously for at least 10 seconds | High |
| R3 | Estimate target state with error < 5% | High |
| R4 | Predict target position 1 second ahead with error < 10% | Medium |
| R5 | Achieve miss distance < 2 meters in simulation | High |
| R6 | System response time < 200 ms | Medium |
| R7 | Operate in wind up to 10 m/s | Medium |
| R8 | Fail-safe if target lost | High |

---

## 7. Interfaces

| Interface | Protocol | Rate |
|---|---|---|
| Camera → Detection | USB / CSI | 30 FPS |
| Detection → Tracking | Python API | 30 FPS |
| Tracking → Estimation | Python API | 30 FPS |
| Estimation → Prediction | Python API | 50 Hz |
| Guidance → Flight Controller | MAVLink | 50 Hz |
| Flight Controller → Dynamics | Simulink | 100 Hz |

---

## 8. Software Stack

| Layer | Tool |
|---|---|
| Perception | Python, OpenCV, PyTorch |
| Estimation | Python, FilterPy |
| Prediction | Python, NumPy |
| Guidance | Python, MATLAB |
| Control | MATLAB/Simulink, ArduPilot |
| Dynamics | MATLAB/Simulink |
| Visualization | MATLAB, Python (Matplotlib) |
| Version Control | Git, GitHub |

---

## 9. Next Steps

1. Write `docs/architecture/frames.md` (coordinate transformations).
2. Write `docs/requirements/requirements.md` (detailed requirements).
3. Build the first Simulink model (3-DOF aircraft).
4. Build the first Python detection script (OpenCV).

---

*Last updated: [9/11/2026]*
