# Robustness tests for scripts/vio_gps_switch.lua (run from C:\Users\funfo\vio\tools, about 15 minutes).
# All runs: 3 min GPS outage in AUTO, wind 6 m/s from 250 deg with turbulence, VIO noise 0.5 m / 0.3 m/s,
# VIO drifting 0.4 m/s towards 45 deg.
#   runaway_old        VIO runs away 40 s into the outage (+3 m/s per s up to 30 m/s) - old script (no checks)
#   runaway_new        the same with the new script
#   slow_runaway_new   a slow runaway (+0.3 m/s per s) - new script
#   wind_real_vio_new  normal VIO, wind shift and a VIO dropout (as wind_real_vio) - new script, must NOT reject
$py = "..\venv\Scripts\python.exe"
$common = @("--param", "SIM_WIND_SPD=6", "--param", "SIM_WIND_DIR=250", "--param", "SIM_WIND_TURB=1",
            "--param", "SIM_VICON_P_SD=0.5", "--param", "SIM_VICON_V_SD=0.3",
            "--param", "VISO_POS_M_NSE=0.5", "--param", "VISO_VEL_M_NSE=0.3", "--vio-drift", "0.4,45")
$runs = [ordered]@{
    "runaway_old"       = @("--script", "..\scripts\archive\vio_gps_switch_v1.lua", "--vio-runaway", "40,3,120,30")
    "runaway_new"       = @("--vio-runaway", "40,3,120,30")
    "slow_runaway_new"  = @("--vio-runaway", "40,0.3,120,30")
    "wind_real_vio_new" = @("--wind-shift", "60,9,310", "--vio-dropout", "100,8")
}
foreach ($name in $runs.Keys) {
    Write-Output "===== $name ====="
    $extra = $runs[$name]
    & $py test_gps_loss.py --name $name @common @extra 2>&1 | Select-String -Pattern "TEST|VSW|script state|ROLL_LIMIT|GPS OFF|GPS back|Lua|rror|Traceback"
    & $py analyze_log.py $name 2>&1 | Select-Object -First 12
}
