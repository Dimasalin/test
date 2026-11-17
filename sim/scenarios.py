"""Named test flights."""
from world import Scenario, story, Airframe
import navi_core as nc

T0 = (1.0, nc.CMD_TAKEOFF, 0, 0, 0)


def standing(seed=1, secs=45.0, **kw):
    """Person stands still; fairy flits around them, then is told to land."""
    return Scenario(name="standing", seed=seed, duration=secs + 12,
                    commands=(T0, (secs, nc.CMD_LAND, 0, 0, 0)), **kw)


def low_battery(seed=1):
    """Starts with a third of a charge: must warn, then land itself."""
    return Scenario(name="low_battery", seed=seed, duration=70.0, soc0=0.36,
                    commands=(T0,))


def flow_loss(seed=1):
    """Optical flow goes blind at t=15 s (dark or glossy floor)."""
    return Scenario(name="flow_loss", seed=seed, duration=30.0,
                    flow_dead_after=15.0, commands=(T0,))


def beacon_dropout(seed=1):
    """Beacon silent for 6 s: wait in place, then carry on."""
    return Scenario(name="beacon_dropout", seed=seed, duration=45.0,
                    uwb_dead=(15.0, 21.0),
                    commands=(T0, (34.0, nc.CMD_LAND, 0, 0, 0)))


def beacon_lost(seed=1):
    """Beacon goes away for good: wait 15 s, then land."""
    return Scenario(name="beacon_lost", seed=seed, duration=50.0,
                    uwb_dead=(15.0, 1e9), commands=(T0,))


def table(seed=1):
    """No beacon (stage-2 build): orbit the take-off spot, part of the orbit
    passes over a 0.75 m table. Altitude must not jump."""
    return Scenario(name="table", seed=seed, duration=50.0,
                    cfg={"uwb_enabled": 0},
                    table=(-1.9, -0.9, 0.1, 1.1, 0.75),
                    commands=(T0, (40.0, nc.CMD_LAND, 0, 0, 0)))


def endurance(seed=1):
    """Hover-and-flit until the battery sends it home."""
    return Scenario(name="endurance", seed=seed, duration=420.0, commands=(T0,))


def home(seed=1, pad=(1.8, -1.2), **kw):
    """Told to go home while the person stands by: find the pad, land on it."""
    return Scenario(name="home", seed=seed, duration=75.0, pad=pad,
                    commands=(T0, (22.0, nc.CMD_HOME, 0, 0, 0)), **kw)


def home_from_pad(seed=1):
    """Takes off from the pad, follows the person across the room, then
    is sent back."""
    return Scenario(name="home_from_pad", seed=seed, duration=95.0, pad=(0.0, 0.0),
                    legs=((10.0, (-3.5, 1.5)), (40.0, (-3.5, -1.0))),
                    commands=(T0, (48.0, nc.CMD_HOME, 0, 0, 0)))


def home_low_battery(seed=1):
    """Nobody says anything: the battery runs down and it takes itself home."""
    return Scenario(name="home_low_battery", seed=seed, duration=130.0, soc0=0.44,
                    pad=(2.2, 1.0), commands=(T0,))


def home_pad_lost(seed=1):
    """Pad beacon dies during the trip home: land where it is."""
    return Scenario(name="home_pad_lost", seed=seed, duration=50.0, pad=(2.5, -1.5),
                    pad_dead_after=23.0,
                    commands=(T0, (22.0, nc.CMD_HOME, 0, 0, 0)))


def home_varied(seed):
    """Go-home on a randomly mis-built airframe, pad in a random place."""
    import numpy as np
    r = np.random.default_rng(2000 + seed)
    ang, dist = r.uniform(0, 2 * np.pi), r.uniform(1.6, 4.0)
    sc = varied(seed)
    sc.name = "home_varied"
    sc.legs = ()
    sc.pad = (-0.7 + dist * np.cos(ang), dist * np.sin(ang))   # >= 1.6 m from the person
    sc.commands = (T0, (20.0, nc.CMD_HOME, 0, 0, 0))
    sc.duration = 80.0
    return sc


def varied(seed):
    """Story flight on a randomly mis-built airframe (Monte Carlo)."""
    import numpy as np
    r = np.random.default_rng(1000 + seed)
    af = Airframe(mass=0.056 * r.uniform(0.93, 1.10),
                  thrust_max=0.40 * r.uniform(0.82, 1.15),
                  drag=0.45 * r.uniform(0.7, 1.4),
                  r_int=0.07 * r.uniform(0.8, 1.5))
    sc = story(seed=seed, airframe=af, flow_scale=r.uniform(0.92, 1.08),
               wind_sigma=r.uniform(0.05, 0.25))
    sc.name = "varied"
    return sc


ALL = {f.__name__: f for f in (story, standing, low_battery, flow_loss,
                               beacon_dropout, beacon_lost, table, endurance,
                               home, home_from_pad, home_low_battery, home_pad_lost)}
