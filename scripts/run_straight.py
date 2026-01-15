# scripts/run_and_visualize.py
import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sim.state import VehicleState
from sim.integrator import run
from models.straight_flat import StraightFlatModel
from visualization.track_anim import animate_history_on_track


def controller_full_throttle(state):
    return 1.0, 0.0


def main():
    state = VehicleState()
    model = StraightFlatModel(
        m=136.1,
        Crr=0.014,
        g=9.81,
        eta=0.85,
        Pmax=1200.0,
        v_eps=0.5
    )
    history = run(model, state, controller_full_throttle, T=120.0, dt=0.01)
    animate_history_on_track(history)


if __name__ == "__main__":
    main()
