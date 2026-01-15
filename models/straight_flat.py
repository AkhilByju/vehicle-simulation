# models/straight_flat.py

class StraightFlatModel:
    """
    v1: Straight + flat terrain
    - Power-limited drive force: F = eta * P / max(v, v_eps)
    - Rolling resistance only: Frr = Crr * m * g
    """

    def __init__(self, m=136.1, Crr=0.014, g=9.81, eta=0.85, Pmax=1200.0, v_eps=0.5):
        self.m = float(m)
        self.Crr = float(Crr)
        self.g = float(g)
        self.eta = float(eta)
        self.Pmax = float(Pmax)
        self.v_eps = float(v_eps)

        self.Frr = self.Crr * self.m * self.g  # constant on flat v1

    def step(self, state, throttle, brake, dt):
        # clamp inputs
        u = min(max(float(throttle), 0.0), 1.0)
        # brake currently ignored in v1 dynamics, but kept for interface
        _b = min(max(float(brake), 0.0), 1.0)

        # power available
        P = u * self.Pmax  # W

        # drive force (power-limited)
        F_drive = (self.eta * P) / max(state.v, self.v_eps)

        # net force
        F_net = F_drive - self.Frr

        # integrate
        state.a = F_net / self.m
        state.v = max(0.0, state.v + state.a * dt)
        state.x = state.x + state.v * dt
        state.t = state.t + dt

        # return extra telemetry fields to log
        return {
            "power_W": P,
            "F_drive_N": F_drive,
            "F_rr_N": self.Frr,
        }
