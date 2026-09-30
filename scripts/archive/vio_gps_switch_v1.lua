--[[
vio_gps_switch.lua - automatic GPS <-> visual odometry (VIO) switch-over for ArduPlane

EKF3 source set 1 = GPS (normal flight), source set 2 = visual odometry (ExternalNav).
  * While armed, if the GPS has been unusable (no fresh 3D fix, too few satellites or poor
    speed accuracy) for VSW_BAD_MS and the visual odometry is healthy, switch to set 2.
  * Once the GPS has been usable again for VSW_GOOD_S seconds, switch back to set 1.
The active set (1 or 2) is reported once a second as NAMED_VALUE_FLOAT "VSW_SRC".

Setup: see params/vio_plane.parm (EK3_SRC1_* = GPS, EK3_SRC2_* = ExternalNav,
EK3_SRC_OPTIONS = 0, VISO_TYPE = 2, SCR_ENABLE = 1). Do not also put an RC switch on
option 90 (EKF source set): the switch and this script would fight over the source set.

Parameters:
  VSW_ENABLE  1 = automatic switching on, 0 = off
  VSW_SATS    minimum satellites for a usable GPS                  (default 6)
  VSW_SACC    maximum GPS speed accuracy in m/s for a usable GPS   (default 1.0)
  VSW_BAD_MS  GPS must be unusable this long before switching      (default 1000 ms)
  VSW_GOOD_S  GPS must be usable this long before switching back   (default 10 s)
--]]

local TABLE_KEY = 73
local UPDATE_MS = 100
local SRC_GPS, SRC_VIO = 0, 1   -- source set indices for ahrs:set_posvelyaw_source_set()
local FIX_TIMEOUT_MS = 500      -- a GPS fix older than this counts as lost
local SEV_WARNING, SEV_INFO = 4, 6

assert(param:add_table(TABLE_KEY, "VSW_", 5), "VSW: could not add parameter table")
assert(param:add_param(TABLE_KEY, 1, "ENABLE", 1), "VSW: could not add VSW_ENABLE")
assert(param:add_param(TABLE_KEY, 2, "SATS", 6), "VSW: could not add VSW_SATS")
assert(param:add_param(TABLE_KEY, 3, "SACC", 1.0), "VSW: could not add VSW_SACC")
assert(param:add_param(TABLE_KEY, 4, "BAD_MS", 1000), "VSW: could not add VSW_BAD_MS")
assert(param:add_param(TABLE_KEY, 5, "GOOD_S", 10), "VSW: could not add VSW_GOOD_S")

local VSW_ENABLE = Parameter("VSW_ENABLE")
local VSW_SATS = Parameter("VSW_SATS")
local VSW_SACC = Parameter("VSW_SACC")
local VSW_BAD_MS = Parameter("VSW_BAD_MS")
local VSW_GOOD_S = Parameter("VSW_GOOD_S")

local bad_since_ms = nil    -- when the GPS became unusable (nil while it is usable)
local good_since_ms = nil   -- when the GPS became usable (nil while it is unusable)
local last_report_ms = 0
local warned_vio = false    -- a "visual odometry unhealthy" warning has been sent

local function gps_usable(now_ms)
    local i = gps:primary_sensor()
    if gps:status(i) < gps.GPS_OK_FIX_3D then
        return false
    end
    if now_ms - gps:last_fix_time_ms(i):toint() > FIX_TIMEOUT_MS then
        return false
    end
    if gps:num_sats(i) < VSW_SATS:get() then
        return false
    end
    local speed_acc = gps:speed_accuracy(i)
    if speed_acc and speed_acc > VSW_SACC:get() then
        return false
    end
    return true
end

local function update()
    if VSW_ENABLE:get() < 1 then
        return update, 1000
    end
    local now_ms = millis():toint()
    local active = ahrs:get_posvelyaw_source_set()

    if gps_usable(now_ms) then
        bad_since_ms = nil
        good_since_ms = good_since_ms or now_ms
    else
        good_since_ms = nil
        bad_since_ms = bad_since_ms or now_ms
    end

    if active == SRC_GPS and arming:is_armed() and bad_since_ms
            and now_ms - bad_since_ms >= VSW_BAD_MS:get() then
        if visual_odom:healthy() then
            ahrs:set_posvelyaw_source_set(SRC_VIO)
            gcs:send_text(SEV_WARNING, "VSW: GPS lost, switched to visual odometry")
            warned_vio = false
        elseif not warned_vio then
            gcs:send_text(SEV_WARNING, "VSW: GPS lost but visual odometry unhealthy")
            warned_vio = true
        end
    elseif active == SRC_VIO then
        if good_since_ms and now_ms - good_since_ms >= VSW_GOOD_S:get() * 1000 then
            ahrs:set_posvelyaw_source_set(SRC_GPS)
            gcs:send_text(SEV_WARNING, "VSW: GPS good again, switched back to GPS")
        elseif not visual_odom:healthy() and not warned_vio then
            gcs:send_text(SEV_WARNING, "VSW: visual odometry unhealthy while in use")
            warned_vio = true
        end
    end

    if now_ms - last_report_ms >= 1000 then
        gcs:send_named_float("VSW_SRC", ahrs:get_posvelyaw_source_set() + 1)
        last_report_ms = now_ms
    end
    return update, UPDATE_MS
end

gcs:send_text(SEV_INFO, "VSW: GPS/visual odometry switch-over loaded")
return update()
