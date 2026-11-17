"""Simulated flight tests. These run the real firmware core (compiled from
firmware/core) against the physics model in sim/world.py."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "sim"))
import navi_core as nc            # noqa: E402
import scenarios                  # noqa: E402
from world import Sim, metrics, story   # noqa: E402


def fly(sc):
    sim = Sim(sc)
    sim.run()
    return sim, metrics(sim)


def states(sim):
    return [nc.STATE_NAMES[s] for s in
            sim.log["state"][np.r_[True, np.diff(sim.log["state"]) != 0]]]


@pytest.mark.parametrize("seed", [1, 2, 3])
def test_standing_companion_is_tracked_tightly(seed):
    sim, m = fly(scenarios.standing(seed=seed))
    assert m["final_state"] == "LANDED"
    assert m["track_rms_m"] < 0.35          # knows where you are
    assert 0.5 < m["mean_dist_m"] < 1.4     # stays close, just overhead
    assert m["min_dist_low_m"] > 0.45       # never in your face
    assert m["max_tilt_deg"] < 25
    assert m["motor_sat_s"] < 0.5
    assert m["faults"] == 0


@pytest.mark.parametrize("seed", [1, 2, 3, 4])
def test_story_flight_stays_bounded_and_safe(seed):
    sim, m = fly(story(seed=seed))
    assert states(sim) == ["LANDED", "TAKEOFF", "FOLLOW", "ATTENTION", "FOLLOW",
                           "TARGET", "FOLLOW", "LAND", "LANDED"]
    # While you walk it trails behind (range-only tracking), but it must not
    # run away, and must stay above head height whenever it is near you.
    assert m["max_dist_follow_m"] < 9.0
    assert m["min_dist_low_m"] > 0.45
    assert m["max_alt_m"] < 2.35
    L = sim.log
    cruising = np.isin(L["state"], [nc.FOLLOW, nc.ATTENTION, nc.TARGET])
    assert L["pos"][cruising, 2].min() > 1.80


def test_usually_back_at_your_side_after_you_stop():
    """Not guaranteed per flight (see DESIGN.md), so test the typical case."""
    ends = []
    for seed in range(1, 8):
        sim, _ = fly(story(seed=seed))
        L = sim.log
        w = (L["t"] > 70) & (L["t"] < 74)        # person stopped at ~58 s
        ends.append(np.linalg.norm(L["pos"][w, :2] - L["person"][w], axis=1).mean())
    assert np.median(ends) < 1.6


def test_attention_chirps_and_flashes_green():
    sim, _ = fly(story(seed=1))
    L = sim.log
    att = L["state"] == nc.ATTENTION
    assert (L["buzzer"][att] > 0).any()
    led = L["led"][att]
    assert (led[:, 1] > led[:, 0]).all() and (led[:, 1] > led[:, 2]).all()
    tgt = L["state"] == nc.TARGET
    led = L["led"][tgt]
    assert (led[:, 0] > led[:, 2]).all()      # yellow: red+green over blue


def test_low_battery_lands_itself():
    sim, m = fly(scenarios.low_battery())
    assert m["final_state"] == "LANDED"
    assert m["faults"] & nc.FAULT_VBAT
    assert "LAND" in states(sim)
    assert m["end_vbat"] > 3.0               # did not run the cell flat
    assert m["touchdown_mps"] < 1.2


def test_optical_flow_loss_descends_level():
    sim, m = fly(scenarios.flow_loss())
    assert states(sim)[-2:] == ["FAILSAFE", "LANDED"]
    assert m["faults"] & nc.FAULT_FLOW
    assert m["touchdown_mps"] < 1.2
    L = sim.log
    fs = L["state"] == nc.FAILSAFE
    assert np.abs(L["rc"][fs, :2]).max() == 0.0     # sticks centred


def test_beacon_dropout_waits_then_resumes():
    sim, m = fly(scenarios.beacon_dropout())
    L = sim.log
    quiet = (L["t"] > 18) & (L["t"] < 21)
    assert (L["state"][quiet] == nc.FOLLOW).all()
    assert np.linalg.norm(L["vel"][quiet, :2], axis=1).mean() < 0.3   # holding still
    after = (L["t"] > 28) & (L["t"] < 33)
    d = np.linalg.norm(L["pos"][after, :2] - L["person"][after], axis=1)
    assert d.mean() < 1.5
    assert m["final_state"] == "LANDED"


def test_beacon_lost_for_good_lands():
    sim, m = fly(scenarios.beacon_lost())
    assert m["final_state"] == "LANDED"
    assert "LAND" in states(sim)


def test_flying_over_a_table_does_not_change_altitude():
    sim, m = fly(scenarios.table())
    L = sim.log
    x0, x1, y0, y1, _ = sim.sc.table
    over = ((L["pos"][:, 0] > x0) & (L["pos"][:, 0] < x1) &
            (L["pos"][:, 1] > y0) & (L["pos"][:, 1] < y1) & (L["state"] == nc.FOLLOW))
    assert over.sum() > 50, "scenario should actually cross the table"
    assert np.abs(L["pos"][over, 2] - 1.5).max() < 0.25
    assert m["final_state"] == "LANDED"


def test_kill_command_cuts_motors_immediately():
    sc = scenarios.standing(secs=20)
    sc.commands = (scenarios.T0, (10.0, nc.CMD_KILL, 0, 0, 0))
    sim, _ = fly(sc)
    L = sim.log
    i = np.searchsorted(L["t"], 10.0) + 2
    assert L["state"][i] == nc.LANDED and L["rc"][i, 3] == 0.0


def test_takeoff_refused_on_flat_battery():
    sc = scenarios.standing(secs=10)
    sc.soc0 = 0.02
    sim, m = fly(sc)
    assert m["airborne_s"] == 0.0
    assert (sim.log["rc"][:, 3] == 0).all()


@pytest.mark.parametrize("seed", range(1, 7))
def test_build_tolerances(seed):
    """Heavier/lighter, weaker/stronger, draggier airframes still fly."""
    sim, m = fly(scenarios.varied(seed))
    assert m["final_state"] == "LANDED"
    assert m["max_dist_follow_m"] < 9.0
    assert m["min_dist_low_m"] > 0.45
    assert m["max_tilt_deg"] < 28
    assert m["motor_sat_s"] < 1.0
