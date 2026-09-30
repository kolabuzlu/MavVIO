--[[
sim_vio_fault.lua - SIMULATOR ONLY: makes the simulated VIO (sim:vicon) drift or run away.
Used by tools/test_gps_loss.py; never put this on a real flight controller.

From the moment VFT_START is set to 1:
  drift    the VIO velocity is off by VFT_DRIFT m/s towards VFT_DRIFT_DIR deg
  runaway  VFT_RUN_T s later the velocity error starts growing by VFT_RUN_ACC m/s every second
           towards VFT_RUN_DIR deg, up to VFT_RUN_MAX m/s - like a VIO that lost track and keeps
           integrating a wrong velocity
The position error is always the integral of the velocity error, so the fake VIO stays
self-consistent, as a real diverging VIO does. VFT_START back to 0 removes the error.
Logs VFT: seconds since start, velocity error N/E (m/s), position error N/E (m).
--]]

local TABLE_KEY = 74
local UPDATE_MS = 50

assert(param:add_table(TABLE_KEY, "VFT_", 7), "VFT: could not add parameter table")
assert(param:add_param(TABLE_KEY, 1, "START", 0), "VFT: could not add VFT_START")
assert(param:add_param(TABLE_KEY, 2, "DRIFT", 0), "VFT: could not add VFT_DRIFT")
assert(param:add_param(TABLE_KEY, 3, "DRIFT_DIR", 0), "VFT: could not add VFT_DRIFT_DIR")
assert(param:add_param(TABLE_KEY, 4, "RUN_T", 0), "VFT: could not add VFT_RUN_T")
assert(param:add_param(TABLE_KEY, 5, "RUN_ACC", 0), "VFT: could not add VFT_RUN_ACC")
assert(param:add_param(TABLE_KEY, 6, "RUN_DIR", 0), "VFT: could not add VFT_RUN_DIR")
assert(param:add_param(TABLE_KEY, 7, "RUN_MAX", 30), "VFT: could not add VFT_RUN_MAX")

local START = Parameter("VFT_START")
local DRIFT = Parameter("VFT_DRIFT")
local DRIFT_DIR = Parameter("VFT_DRIFT_DIR")
local RUN_T = Parameter("VFT_RUN_T")
local RUN_ACC = Parameter("VFT_RUN_ACC")
local RUN_DIR = Parameter("VFT_RUN_DIR")
local RUN_MAX = Parameter("VFT_RUN_MAX")

local start_ms = nil

local function set_error(vn, ve, pn, pe)
    param:set("SIM_VICON_VGLI_X", vn)
    param:set("SIM_VICON_VGLI_Y", ve)
    param:set("SIM_VICON_GLIT_X", pn)
    param:set("SIM_VICON_GLIT_Y", pe)
end

-- runaway speed error (m/s) and distance error (m) along VFT_RUN_DIR, t seconds after VFT_START
local function runaway(t)
    local acc, vmax = RUN_ACC:get(), RUN_MAX:get()
    local s = t - RUN_T:get()
    if acc <= 0 or s <= 0 then
        return 0, 0
    end
    local t_full = vmax / acc
    if s < t_full then
        return acc * s, 0.5 * acc * s * s
    end
    return vmax, 0.5 * vmax * t_full + vmax * (s - t_full)
end

local function update()
    local now_ms = millis():toint()
    if START:get() < 1 then
        if start_ms then
            set_error(0, 0, 0, 0)
            start_ms = nil
        end
        return update, UPDATE_MS
    end
    start_ms = start_ms or now_ms
    local t = (now_ms - start_ms) / 1000
    local drift, ddir, rdir = DRIFT:get(), math.rad(DRIFT_DIR:get()), math.rad(RUN_DIR:get())
    local rv, rp = runaway(t)
    local vn = drift * math.cos(ddir) + rv * math.cos(rdir)
    local ve = drift * math.sin(ddir) + rv * math.sin(rdir)
    local pn = drift * t * math.cos(ddir) + rp * math.cos(rdir)
    local pe = drift * t * math.sin(ddir) + rp * math.sin(rdir)
    set_error(vn, ve, pn, pe)
    logger:write("VFT", "T,VN,VE,PN,PE", "fffff", t, vn, ve, pn, pe)
    return update, UPDATE_MS
end

return update()
