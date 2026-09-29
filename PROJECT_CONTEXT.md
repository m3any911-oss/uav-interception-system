# Talon C-UAS Project Context & Technical Baseline

## 1. Project Overview & Scope
The Talon Counter-Unmanned Aerial System (C-UAS) is an integrated mechatronic platform designed to intercept mini-UAV threats. The system couples an ultra-compact pneumatic launcher with a fixed-wing autonomous interceptor drone guided via MAVLink.

## 2. Ultra-Compact Pneumatic Launcher Baseline
* **Rail Length ($d$):** 1.0 m (Ultra-compact man-portable)
* **Moving Mass ($m$):** 3.5 kg
* **Target Launch Velocity ($V_{\text{launch}}$):** 18.0 m/s
* **Design Acceleration ($a$):** 162.0 m/s² (16.51 g)
* **Stroke Pulse Duration ($t_{\text{launch}}$):** 111.1 ms
* **Operating Pressure ($P_{\text{op}}$):** 4.33 bar (62.8 PSI)
* **Accumulator Volume ($V_{\text{acc}}$):** 5.0 Liters @ 8.0 bar pre-charge

## 3. Airframe & Dynamic Assumptions (Talon Fixed-Wing)
* **Take-off Mass ($m$):** 3.5 kg
* **Wing Area ($S$):** 0.60 m²
* **Mean Aerodynamic Chord ($c$):** 0.25 m
* **Pitch Moment of Inertia ($I_{yy}$):** 0.20 kg·m²
* **Aerodynamic Coefficients (Baseline Longitudinal):**
  * $C_{L0} = 0.2430$, $C_{L\alpha} = 1.1385$, $C_{L\delta_e} = 0.30$, $C_{Lq} = 4.8052$
  * $C_{D0} = 0.00976$, $C_{D\alpha^2} = 0.01$
  * $C_{m0} = -0.02$, $C_{m\alpha} = -0.6281$, $C_{m\delta_e} = -1.20$, $C_{mq} = -20.2775$

## 4. Primary Coordinate Conventions & Units
* **Spatial Reference:** North-East-Down (NED) Frame. Altitude $h = -Z$.
* **Body Reference:** Forward-Right-Down (FRD) Frame.
* **SI Units:** Meters (m), Seconds (s), Radians (rad), Newtons (N), Kilograms (kg).
