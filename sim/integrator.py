# sim/integrator.py

def run(model, state, controller, T: float, dt: float):
    """
    Generic time integrator.
    - model.step(state, throttle, brake, dt) updates the state in-place
    - controller(state) returns (throttle, brake)
    Returns: list[dict] history
    """
    history = []

    # record initial state as well (optional)
    history.append({
        "t": state.t,
        "x_m": state.x,
        "v_mps": state.v,
        "a_mps2": state.a,
        "throttle": 0.0,
        "brake": 0.0,
    })

    while state.t < T:
        throttle, brake = controller(state)
        outputs = model.step(state, throttle, brake, dt) or {}

        row = {
            "t": state.t,
            "x_m": state.x,
            "v_mps": state.v,
            "a_mps2": state.a,
            "throttle": float(throttle),
            "brake": float(brake),
            **outputs,
        }
        history.append(row)

    return history
