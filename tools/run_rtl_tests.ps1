# RTL tests for scripts/vio_gps_switch.lua (run from C:\Users\funfo\vio\tools, about 15 minutes).
# All runs: 3 min GPS outage in AUTO, wind 6 m/s from 250 deg with turbulence.
#   rtl_runaway       VIO runs away 40 s into the outage -> rejected -> RTL 10 s later
#   rtl_no_vio        no VIO at all, wind shift after 60 s -> RTL 11 s after GPS loss, then circling on dead reckoning
#   rtl_normal_vio    normal VIO, wind shift, 8 s dropout (as wind_real_vio_new) -> must NOT RTL
#   rtl_long_dropout  normal VIO that drops out for 20 s -> RTL during the dropout, then home on VIO
$py = "..\venv\Scripts\python.exe"
$wind = @("--param", "SIM_WIND_SPD=6", "--param", "SIM_WIND_DIR=250", "--param", "SIM_WIND_TURB=1")
$vio = @("--param", "SIM_VICON_P_SD=0.5", "--param", "SIM_VICON_V_SD=0.3",
         "--param", "VISO_POS_M_NSE=0.5", "--param", "VISO_VEL_M_NSE=0.3", "--vio-drift", "0.4,45")
$runs = [ordered]@{
    "rtl_runaway"      = $vio + @("--vio-runaway", "40,3,120,30")
    "rtl_no_vio"       = @("--no-vio", "--wind-shift", "60,9,310")
    "rtl_normal_vio"   = $vio + @("--wind-shift", "60,9,310", "--vio-dropout", "100,8")
    "rtl_long_dropout" = $vio + @("--vio-dropout", "60,20")
}
foreach ($name in $runs.Keys) {
    Write-Output "===== $name ====="
    $extra = $runs[$name]
    & $py test_gps_loss.py --name $name @wind @extra 2>&1 | Select-String -Pattern "TEST|VSW|script state|flight mode|ROLL_LIMIT|GPS OFF|GPS back|Lua|rror|Traceback"
    & $py analyze_log.py $name 2>&1 | Select-Object -First 14
    & $py plot_run.py $name 2>&1
}
