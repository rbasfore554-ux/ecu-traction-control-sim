import numpy as np
import matplotlib.pyplot as plt

# =====================================================================
# SIMULATION CONFIGURATION & VEHICLE PARAMETERS
# =====================================================================
TOTAL_TIME = 5.0      # Simulation duration (seconds)
SIM_HZ = 1000         # Physics simulation resolution (1ms steps)
ECU_HZ = 100          # ECU loop operational frequency (10ms steps)

PHYSICS_STEPS = int(TOTAL_TIME * SIM_HZ)
ECU_STRIDE = int(SIM_HZ / ECU_HZ)

# Vehicle Physical Properties (Approximating a GT3 Class Race Car)
MASS = 1300.0         # Vehicle Mass (kg)
GREx = 12.0           # Overall Gear Ratio (Engine to Wheels)
R_TYRE = 0.33         # Tyre Rolling Radius (meters)
I_WHEEL = 1.5         # Rotational Inertia of Rear Drive Assembly (kg*m^2)
PEAK_TORQUE = 650.0   # Maximum available engine torque (Nm)

# =====================================================================
# INITIAL STATE VARIABLES
# =====================================================================
v_car = 12.0          # Initial true vehicle velocity exiting corner (m/s)
w_rear = v_car / R_TYRE # Initial rear wheel angular velocity (rad/s)
w_front = v_car / R_TYRE # Initial front wheel angular velocity (rad/s)

# Control System Parameters (PID Target Slip)
TARGET_SLIP = 0.10     # Optimal grip window peak at 10% wheel slip
Kp = 8.5               # Proportional Gain
Ki = 15.0              # Integral Gain
Kd = 0.1               # Derivative Gain

# Pre-allocated arrays for data logging
log_time = np.zeros(PHYSICS_STEPS)
log_v_car = np.zeros(PHYSICS_STEPS)
log_v_rear = np.zeros(PHYSICS_STEPS)
log_slip = np.zeros(PHYSICS_STEPS)
log_torque_req = np.zeros(PHYSICS_STEPS)
log_torque_act = np.zeros(PHYSICS_STEPS)

integral_error = 0.0
last_error = 0.0
tcs_cut_factor = 1.0  # 1.0 = full power, 0.0 = complete cut

# =====================================================================
# MAIN SIMULATION LOOP (Forward Euler Integration)
# =====================================================================
for i in range(PHYSICS_STEPS):
    t = i / SIM_HZ
    log_time[i] = t
    
    # 1. Driver Intent Model (Simulating aggressive 100% throttle application)
    raw_driver_torque = PEAK_TORQUE if t > 0.2 else (PEAK_TORQUE * (t / 0.2))
    
    # 2. ECU Control Loop (Executes strictly at 100Hz)
    if i % ECU_STRIDE == 0:
        # Sensor Reading Emulation (Adding minor measurement noise)
        sensor_v_front = v_car + np.random.normal(0, 0.02)
        sensor_v_rear = (w_rear * R_TYRE) + np.random.normal(0, 0.02)
        
        # Calculate Longitudinal Wheel Slip Ratio
        if sensor_v_front > 0.5:
            current_slip = (sensor_v_rear - sensor_v_front) / sensor_v_front
        else:
            current_slip = 0.0
            
        # PID Error evaluation relative to optimal grip threshold
        error = current_slip - TARGET_SLIP
        
        if current_slip > TARGET_SLIP:
            integral_error += error * (1.0 / ECU_HZ)
            derivative_error = (error - last_error) * ECU_HZ
            pid_output = (Kp * error) + (Ki * integral_error) + (Kd * derivative_error)
            
            # Translate PID output to a torque mitigation factor (clamped between 0 and 1)
            tcs_cut_factor = np.clip(1.0 - pid_output, 0.15, 1.0)
        else:
            # Re-engage power smoothly if slip is under control
            integral_error = max(0.0, integral_error - 0.1)
            tcs_cut_factor = min(1.0, tcs_cut_factor + 0.05)
            
        last_error = error

    # 3. Dynamic Physics Core (1000Hz execution)
    actual_engine_torque = raw_driver_torque * tcs_cut_factor
    wheel_torque = actual_engine_torque * GREx
    
    # Non-linear Tyre Force Curve (Magic Formula Approximation with 0.4 Wet Multiplier)
    true_slip = ((w_rear * R_TYRE) - v_car) / max(0.1, v_car)
    mu = 0.4 * 1.65 * np.sin(1.65 * np.arctan(10.0 * true_slip)) 
    
    # Vertical Load per drive wheel (Static weight + dynamic rearward transfer)
    f_vertical = (MASS * 9.81 * 0.5) + (MASS * 1.5 * (wheel_torque / R_TYRE / MASS) / 2.8)
    f_traction = mu * f_vertical
    
    # Rotational Acceleration of the Driven Wheel
    dw_rear = (wheel_torque - (f_traction * R_TYRE)) / I_WHEEL
    w_rear += dw_rear * (1.0 / SIM_HZ)
    w_rear = max(v_car / R_TYRE, w_rear) # Prevent negative slip physically
    
    # Linear Acceleration of the Car body
    dv_car = f_traction / MASS
    v_car += dv_car * (1.0 / SIM_HZ)
    
    # Logging data vectors
    log_v_car[i] = v_car
    log_v_rear[i] = w_rear * R_TYRE
    log_slip[i] = true_slip
    log_torque_req[i] = raw_driver_torque
    log_torque_act[i] = actual_engine_torque

# =====================================================================
# DATA VISUALIZATION GENERATION
# =====================================================================
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

ax1.plot(log_time, log_v_rear, label="Driven Wheel Speed (Rear)", color="crimson", linewidth=2)
ax1.plot(log_time, log_v_car, label="True Vehicle Speed (Ground)", color="black", linestyle="--", linewidth=2)
ax1.set_ylabel("Velocity [m/s]", fontsize=11, fontweight="bold")
ax1.legend(loc="upper left")
ax1.grid(True, alpha=0.3)
ax1.set_title("ECU Closed-Loop Traction Control Simulation (100Hz Control Strategy)", fontsize=13, fontweight="bold")

ax2.plot(log_time, log_slip * 100, color="darkorange", linewidth=2, label="Calculated Slip Ratio")
ax2.axhline(TARGET_SLIP * 100, color="green", linestyle=":", label="Target Optimal Grip Window")
ax2.set_ylabel("Wheel Slip [%]", fontsize=11, fontweight="bold")
ax2.legend(loc="upper left")
ax2.grid(True, alpha=0.3)

ax3.plot(log_time, log_torque_req, label="Driver Requested Torque", color="gray", linestyle=":")
ax3.plot(log_time, log_torque_act, label="ECU Modulated Torque Output", color="navy", linewidth=2)
ax3.set_xlabel("Time [Seconds]", fontsize=11, fontweight="bold")
ax3.set_ylabel("Torque [Nm]", fontsize=11, fontweight="bold")
ax3.legend(loc="upper left")
ax3.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("tcs_simulation_results.png", dpi=300)
plt.show()
