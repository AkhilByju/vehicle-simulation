import numpy as np

def simulate_straight_flat(
    T=30.0,          # total simulation time (s)
    dt=0.01,         # timestep (s)
    throttle=1.0,    # constant throttle in [0,1]
    brake=0.0,       # constant brake in [0,1] (unused for now)
    m=136.1,         # kg
    g=9.81,          # m/s^2
    Crr=0.014,       # rolling resistance coefficient
    eta=0.85,        # drivetrain efficiency
    Pmax=1200.0,     # W (tunable)
    v_eps=0.5        # m/s (avoid divide-by-zero)

):
        
    # time array
    n = int(T / dt) + 1
    t = np.linspace(0.0, T, n)

    # state arrays
    x = np.zeros(n)         # pos (m)
    v = np.zeros(n)         # speed (m/s)
    a = np.zeros(n)         # acceleration (m/s^2)

    # Telemmtry Arrays
    P = np.zeros(n)         # power (W)
    F_drive = np.zeros(n)   # drive force (N)
    F_rr = np.zeros(n)      # rolling resistance (N)

    # Constant Rolling Resistance
    Frr_const = Crr * m * g

    for k in range(n - 1):
        # Inputs
        u = float(np.clip(throttle, 0.0, 1.0))

        # no brake
        _b = float(np.clip(brake, 0.0, 1.0))

        # Power Calculations:
        Pk = u * Pmax
        P[k] = Pk

        F_rr[k] = Frr_const

        denom_v = max(v[k], v_eps)
        Fd = (eta * Pk) / denom_v
        F_drive[k] = Fd
        
        # Net force and accleration
        F_net = Fd - Frr_const
        a[k] = F_net / m

                # Integrate (explicit Euler)
        v_next = v[k] + a[k] * dt
        v[k + 1] = max(0.0, v_next)

        x[k + 1] = x[k] + v[k + 1] * dt

    # Fill last sample telemetry nicely
    P[-1] = np.clip(throttle, 0.0, 1.0) * Pmax
    F_rr[-1] = Frr_const
    F_drive[-1] = (eta * P[-1]) / max(v[-1], v_eps)
    a[-1] = (F_drive[-1] - Frr_const) / m



    
    return {
        "t": t,
        "x": x,
        "v": v,
        "a": a,
        "P": P,
        "F_drive": F_drive,
        "F_rr": F_rr,
    }
    
if __name__ == "__main__":
    data = simulate_straight_flat(T=60.0, dt=0.01, throttle=1.0)
    print("Final speed (m/s):", data["v"][-1])
    print("Final speed (mph):", data["v"][-1] * 2.23694)
    print("Final distance (m):", data["x"][-1])