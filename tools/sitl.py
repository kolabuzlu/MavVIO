"""Start ArduPilot's plane simulator in this project's own folder and connect to it.

Two builds: "mp" = Mission Planner's ArduPlane.exe (4.8.0-dev), "wsl" = the ArduPlane 4.7.1 simulator
built in Ubuntu by setup/build_ardupilot_gazebo.sh. Either runs as instance 1 (MAVLink on TCP 5770,
reachable from Windows as 127.0.0.1 in both cases), so it never clashes with a simulator started from
Mission Planner or Gazebo (TCP 5760), and keeps its parameters, logs and Lua scripts in vio/sitl
(~/sitl_vio inside Ubuntu for the Ubuntu build) - Mission Planner's own sitl/plane folder is never touched.
"""
import shutil
import subprocess
import time
from pathlib import Path

from pymavlink import mavutil

ROOT = Path(__file__).resolve().parent.parent
MP_SITL = Path.home() / "OneDrive" / "Documents" / "Mission Planner" / "sitl"
EXE = MP_SITL / "ArduPlane.exe"
PLANE_DEFAULTS = MP_SITL / "models" / "plane.parm"
WORKDIR = ROOT / "sitl"
HOME = "-35.363261,149.165230,584,353"  # ArduPilot's standard SITL airfield (CMAC)
INSTANCE = 1
PORT = 5760 + 10 * INSTANCE
WSL_DISTRO = "Ubuntu-22.04"
WSL_ARDUPILOT = "/home/kolabuzlu/ardupilot"
WSL_PLANE_DEFAULTS = WSL_ARDUPILOT + "/Tools/autotest/models/plane.parm"
WSL_WORKDIR = "/home/kolabuzlu/sitl_vio"   # on Ubuntu's own disk: with its folder on /mnt/c the simulator dies at start


def workdir(build="mp"):
    """Folder the simulator runs in: parameters (eeprom.bin), logs/ and scripts/ (as Windows sees it)."""
    if build == "mp":
        return WORKDIR
    return Path("//wsl.localhost/" + WSL_DISTRO + WSL_WORKDIR)


def wsl_path(path):
    """C:/Users/... -> /mnt/c/Users/... (how Ubuntu sees a Windows path)."""
    p = Path(path).resolve()
    return "/mnt/" + p.drive[0].lower() + p.as_posix()[2:]


def start(extra_defaults=(), speedup=1, wipe=True, vicon=True, build="mp"):
    """Start ArduPlane SITL; returns the process. Console output goes to console.log in its folder."""
    folder = workdir(build)
    folder.mkdir(exist_ok=True)
    (folder / "scripts").mkdir(exist_ok=True)
    args = ["--model", "plane", "--home", HOME, "--speedup", str(speedup), "--instance", str(INSTANCE)]
    if vicon:
        args += ["--serial5", "sim:vicon:"]
    if wipe:
        args.append("--wipe")
    if build == "mp":
        defaults = ",".join(str(p) for p in (PLANE_DEFAULTS, *extra_defaults))
        log = open(folder / "console.log", "w")
        return subprocess.Popen([str(EXE), *args, "--defaults", defaults], cwd=folder, stdout=log,
                                stderr=subprocess.STDOUT)
    # through setup/run_sitl_wsl.sh, which writes console.log itself (see there for why)
    defaults = ",".join([WSL_PLANE_DEFAULTS] + [wsl_path(p) for p in extra_defaults])
    return subprocess.Popen(["wsl", "-d", WSL_DISTRO, "--", "bash", wsl_path(ROOT / "setup" / "run_sitl_wsl.sh"),
                             WSL_WORKDIR, *args, "--defaults", defaults],
                            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def connect(timeout=60):
    """Connect to the running simulator over TCP and wait for its first heartbeat."""
    deadline = time.time() + timeout
    while True:
        try:
            m = mavutil.mavlink_connection(f"tcp:127.0.0.1:{PORT}", source_system=255, autoreconnect=True)
            m.wait_heartbeat(timeout=10)
            return m
        except Exception:
            if time.time() > deadline:
                raise
            time.sleep(1)


def stop(proc, build="mp"):
    """Stop the simulator process."""
    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
    if build != "mp":
        # the Linux process can outlive wsl.exe; "[a]" keeps pkill from matching its own shell
        subprocess.run(["wsl", "-d", WSL_DISTRO, "--", "pkill", "-f", f"[a]rduplane.*--instance {INSTANCE}"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def install_scripts(*paths, build="mp"):
    """Make exactly these Lua scripts the ones the simulator runs (scripts/ in its folder)."""
    folder = workdir(build) / "scripts"
    folder.mkdir(parents=True, exist_ok=True)
    for old in folder.glob("*.lua"):
        old.unlink()
    for p in paths:
        shutil.copy(p, folder)


def set_rates(m, rates_hz):
    """Ask for the given MAVLink messages ({message id: rate in Hz})."""
    for msg_id, hz in rates_hz.items():
        m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                0, msg_id, 1e6 / hz, 0, 0, 0, 0, 0)


def get_param(m, name, timeout=3):
    """Read one parameter; None if the vehicle has no parameter of that name."""
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is not None and msg.param_id == name:
            return msg.param_value
    return None


def set_param(m, name, value, timeout=3):
    """Set one parameter and confirm the vehicle accepted it."""
    for _ in range(3):
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(), value,
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is not None and msg.param_id == name and abs(msg.param_value - value) < 1e-4:
                return
    raise RuntimeError(f"could not set {name} = {value}")
