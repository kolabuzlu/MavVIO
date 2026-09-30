#!/usr/bin/env python3
"""Makes ArduPilot's Gazebo plugin (~/ardupilot_gazebo) send the airspeed through the wind, then rebuild it.

In Gazebo the Zephyr's lift and drag use its speed through the air - the world's <wind> (changed by the
WindEffects system, setup/gz_set_wind.sh) - but ArduPilot's simulator treats a JSON model's wind as zero,
so its simulated airspeed sensor read the ground speed: in wind the plane flew on a wrong airspeed (and
downwind could stall). With this change the plugin sends "airspeed" = |velocity - wind| in its JSON state,
which SITL then uses for the airspeed sensor, as a real pitot would see it.
Applied to a clean checkout each time (git checkout of the file first); setup/ardupilot_gazebo_airspeed.patch
keeps a copy of the change.

Run inside Ubuntu:  python3 /mnt/c/Users/funfo/vio/setup/ardupilot_gazebo_airspeed.py
Then rebuild:       cd ~/ardupilot_gazebo/build && make -j2
"""
import subprocess
from pathlib import Path

REPO = Path.home() / "ardupilot_gazebo"
SRC = REPO / "src/ArduPilotPlugin.cc"
MARK = "vio project"


def patch(path, old, new):
    text = path.read_text()
    assert text.count(old) == 1, f"{path.name}: expected text not found once: {old[:60]!r}"
    path.write_text(text.replace(old, new))


subprocess.run(["git", "-C", str(REPO), "checkout", "--", "src/ArduPilotPlugin.cc"], check=True)
patch(SRC, "#include <gz/sim/components/World.hh>\n",
      "#include <gz/sim/components/World.hh>\n#include <gz/sim/components/Wind.hh>\n")
patch(SRC, """    gz::math::Vector3d velWldA = wldAToWldG.Rot() * velWldG + wldAToWldG.Pos();
""", """    gz::math::Vector3d velWldA = wldAToWldG.Rot() * velWldG + wldAToWldG.Pos();

    // """ + MARK + """: airspeed through the world's wind (what the lift and drag use), for ArduPilot's
    // simulated airspeed sensor - SITL itself takes a JSON model's wind as zero
    gz::math::Vector3d windWldG = gz::math::Vector3d::Zero;
    const auto windEntity = _ecm.EntityByComponents(gz::sim::components::Wind());
    if (windEntity != gz::sim::kNullEntity)
    {
        const auto *windVel =
            _ecm.Component<gz::sim::components::WorldLinearVelocity>(windEntity);
        if (windVel != nullptr)
        {
            windWldG = windVel->Data();
        }
    }
    const double airspeed = (velWldG - windWldG).Length();
""")
patch(SRC, """    writer.Key("velocity");
    writer.StartArray();
    writer.Double(velWldA.X());
    writer.Double(velWldA.Y());
    writer.Double(velWldA.Z());
    writer.EndArray();
""", """    writer.Key("velocity");
    writer.StartArray();
    writer.Double(velWldA.X());
    writer.Double(velWldA.Y());
    writer.Double(velWldA.Z());
    writer.EndArray();

    // """ + MARK + """: see above
    writer.Key("airspeed");
    writer.Double(airspeed);
""")
subprocess.run(f"git -C {REPO} diff > /mnt/c/Users/funfo/vio/setup/ardupilot_gazebo_airspeed.patch", shell=True, check=True)
print("patched - now rebuild the plugin")
