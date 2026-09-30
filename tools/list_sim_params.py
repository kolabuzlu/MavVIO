"""List the simulator's SIM_VICON_*, SIM_WIND_*, SIM_ARSPD_* and VISO_* parameters with their values."""
import sys
import time

from pymavlink import mavutil

import sitl

PREFIXES = tuple(sys.argv[1:]) or ("SIM_VICON", "SIM_WIND", "SIM_ARSPD", "VISO_")
proc = sitl.start(extra_defaults=[sitl.ROOT / "params" / "vio_plane.parm"])
try:
    m = sitl.connect()
    time.sleep(5)
    m.mav.param_request_list_send(m.target_system, m.target_component)
    params, total, last = {}, None, time.time()
    while time.time() - last < 5:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg is None:
            continue
        params[msg.param_id] = msg.param_value
        total, last = msg.param_count, time.time()
        if total and len(params) >= total:
            break
    print(f"received {len(params)} of {total} parameters")
    for name in sorted(n for n in params if n.startswith(PREFIXES)):
        print(f"  {name:18} {params[name]:g}")
finally:
    sitl.stop(proc)
