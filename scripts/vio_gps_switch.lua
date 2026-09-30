--[[
vio_gps_switch.lua - automatic GPS <-> visual odometry (VIO) switch-over for ArduPlane

EKF3 source set 1 = GPS (normal flight), source set 2 = visual odometry (ExternalNav).
  * While armed, if the GPS has been unusable (no fresh 3D fix, too few satellites or poor
    speed accuracy) for VSW_BAD_MS, switch to set 2 if the VIO is healthy - otherwise stay on
    set 1, where without GPS the EKF dead-reckons on airspeed and compass.
  * With VSW_GPS_RTL = 1 (default) the switch to the VIO also sends the plane home (RTL) - if it is
    flying itself (AUTO, GUIDED, LOITER, TAKEOFF) and not landing; the VIO is the backup that gets it
    back. With 0 the plane carries on with what it was doing, navigating on the VIO.
  * With VSW_WEAVE > 0 (off by default) it flies home in gentle S-turns instead of a straight line:
    GUIDED, towards points VSW_WEAVE_D apart along the line home and VSW_WEAVE either side of it in turn,
    then RTL for the last 1.5 x VSW_WEAVE_D. Flying straight at a constant speed hides the scale from a
    single camera; turning shows it (simulator: less drift per km). If the VIO is rejected or the GPS
    returns, it goes straight to RTL; if the pilot changes the mode, it stops and leaves it to the pilot.
  * While on VIO, two checks watch for a VIO that has gone wrong (diverged). If either fails for
    VSW_REJ_T seconds, the VIO is rejected and the EKF goes back to set 1 (dead reckoning) until
    the GPS returns:
      - IMU check: the EKF's velocity innovation test ratio (how far the VIO velocity is from what
        the IMU predicts, 1 = the EKF's own rejection gate) stays above VSW_INNOV. This catches a
        fast runaway within seconds, before the EKF gives in and resets itself onto the bad VIO.
      - airspeed check: the EKF ground velocity (which follows the VIO) is more than VSW_VEL_ERR
        away from the airspeed along the nose plus the wind remembered at the switch-over. This
        catches a slow runaway the EKF has followed.
  * While the VIO is navigating, the bank limit ROLL_LIMIT_DEG is lowered to VSW_ROLL, so turns
    stay gentle and a downward camera keeps seeing the ground below. The RTL circle (RTL_RADIUS)
    is widened to what the plane can fly at that bank downwind (AIRSPEED_CRUISE + wind + 2 m/s).
    Both changes are never saved and are undone when the GPS is back or the VIO is rejected - in
    dead reckoning there is no camera to protect, and RTL circles home tighter with normal bank.
  * With neither GPS nor usable VIO (VIO rejected, missing, or in a dropout) for VSW_RTL_S seconds,
    the plane returns home (RTL) - if it is flying itself (AUTO, GUIDED, LOITER, TAKEOFF) and not
    landing. This happens once per GPS outage: if the pilot switches away from RTL, the script
    leaves it at that. The plane stays in RTL when the GPS comes back.
  * Once the GPS has been usable again for VSW_GOOD_S seconds, go back to normal.
Once a second it reports NAMED_VALUE_FLOAT VSW_SRC (active source set), VSW_ST (0 = GPS,
1 = VIO, 2 = dead reckoning), VSW_IVR (velocity innovation test ratio, -1 = not available) and
VSW_VERR (airspeed check mismatch in m/s; -1 = not on VIO or cannot judge). The flight log gets
the same plus the remembered wind, the bank limit and the RTL radius as VSW, 10 times a second.

Setup: ArduPlane 4.5 or newer (parameter names ROLL_LIMIT_DEG and AIRSPEED_CRUISE); tested in SITL
4.8.0-dev and 4.7.1. See params/vio_plane.parm (EK3_SRC1_* = GPS, EK3_SRC2_* = ExternalNav, EK3_SRC_OPTIONS = 0,
VISO_TYPE = 1, SCR_ENABLE = 1). The airspeed check needs a real airspeed sensor (ARSPD_USE = 1)
and a wind estimate at the moment of the switch; without them only the IMU check works. Do not
also put an RC switch on option 90 (EKF source set): the switch and this script would fight.

Parameters:
  VSW_ENABLE   1 = automatic switching on, 0 = off
  VSW_SATS     minimum satellites for a usable GPS                    (default 6)
  VSW_SACC     maximum GPS speed accuracy in m/s for a usable GPS     (default 1.0)
  VSW_BAD_MS   GPS must be unusable this long before switching        (default 1000 ms)
  VSW_GOOD_S   GPS must be usable this long before switching back     (default 10 s)
  VSW_INNOV    IMU check: reject above this test ratio                (default 1.0, 0 = off)
  VSW_VEL_ERR  airspeed check: reject above this mismatch in m/s      (default 12, 0 = off)
  VSW_REJ_T    a check must fail this many seconds in a row to reject (default 1)
  VSW_ROLL     bank limit in degrees while the VIO navigates          (default 20, 0 = unchanged)
  VSW_RTL_S    RTL after this many seconds without GPS and VIO        (default 10, 0 = never)
  VSW_GPS_RTL  1 = return home (RTL) when switching to the VIO on GPS loss, 0 = carry on (default 1)
  VSW_WEAVE    fly home in S-turns this many metres either side of the line (default 0 = straight RTL)
  VSW_WEAVE_D  ... with a turn every this many metres along it                (default 300)
--]]

local TABLE_KEY = 73
local UPDATE_MS = 100
local SRC_GPS, SRC_VIO = 0, 1   -- source set indices for ahrs:set_posvelyaw_source_set()
local FIX_TIMEOUT_MS = 500      -- a GPS fix older than this counts as lost
local SEV_WARNING, SEV_INFO = 4, 6
local ST_GPS, ST_VIO, ST_DR = 0, 1, 2
local MODE_RTL = 11
local MODE_GUIDED = 15
local SELF_FLYING = {[10] = true, [12] = true, [13] = true, [15] = true}   -- AUTO, LOITER, TAKEOFF, GUIDED

assert(param:add_table(TABLE_KEY, "VSW_", 13), "VSW: could not add parameter table")
assert(param:add_param(TABLE_KEY, 1, "ENABLE", 1), "VSW: could not add VSW_ENABLE")
assert(param:add_param(TABLE_KEY, 2, "SATS", 6), "VSW: could not add VSW_SATS")
assert(param:add_param(TABLE_KEY, 3, "SACC", 1.0), "VSW: could not add VSW_SACC")
assert(param:add_param(TABLE_KEY, 4, "BAD_MS", 1000), "VSW: could not add VSW_BAD_MS")
assert(param:add_param(TABLE_KEY, 5, "GOOD_S", 10), "VSW: could not add VSW_GOOD_S")
assert(param:add_param(TABLE_KEY, 6, "INNOV", 1.0), "VSW: could not add VSW_INNOV")
assert(param:add_param(TABLE_KEY, 7, "VEL_ERR", 12), "VSW: could not add VSW_VEL_ERR")
assert(param:add_param(TABLE_KEY, 8, "REJ_T", 1), "VSW: could not add VSW_REJ_T")
assert(param:add_param(TABLE_KEY, 9, "ROLL", 20), "VSW: could not add VSW_ROLL")
assert(param:add_param(TABLE_KEY, 10, "RTL_S", 10), "VSW: could not add VSW_RTL_S")
assert(param:add_param(TABLE_KEY, 11, "GPS_RTL", 1), "VSW: could not add VSW_GPS_RTL")
assert(param:add_param(TABLE_KEY, 12, "WEAVE", 0), "VSW: could not add VSW_WEAVE")
assert(param:add_param(TABLE_KEY, 13, "WEAVE_D", 300), "VSW: could not add VSW_WEAVE_D")

local VSW_ENABLE = Parameter("VSW_ENABLE")
local VSW_SATS = Parameter("VSW_SATS")
local VSW_SACC = Parameter("VSW_SACC")
local VSW_BAD_MS = Parameter("VSW_BAD_MS")
local VSW_GOOD_S = Parameter("VSW_GOOD_S")
local VSW_INNOV = Parameter("VSW_INNOV")
local VSW_VEL_ERR = Parameter("VSW_VEL_ERR")
local VSW_REJ_T = Parameter("VSW_REJ_T")
local VSW_ROLL = Parameter("VSW_ROLL")
local VSW_RTL_S = Parameter("VSW_RTL_S")
local VSW_GPS_RTL = Parameter("VSW_GPS_RTL")
local VSW_WEAVE = Parameter("VSW_WEAVE")
local VSW_WEAVE_D = Parameter("VSW_WEAVE_D")
local ROLL_LIMIT = Parameter("ROLL_LIMIT_DEG")
local RTL_RADIUS = Parameter("RTL_RADIUS")
local WP_LOITER_RAD = Parameter("WP_LOITER_RAD")
local AIRSPEED_CRUISE = Parameter("AIRSPEED_CRUISE")

local state = ST_GPS
local bad_since_ms = nil        -- when the GPS became unusable (nil while it is usable)
local good_since_ms = nil       -- when the GPS became usable (nil while it is unusable)
local innov_since_ms = nil      -- when the IMU check started failing
local speed_since_ms = nil      -- when the airspeed check started failing
local vio_rejected = false      -- the VIO failed a check during this GPS outage
local wind_n, wind_e = nil, nil -- wind remembered when the VIO took over (nil: no estimate)
local roll_saved = nil          -- ROLL_LIMIT_DEG before it was lowered
local radius_saved = nil        -- RTL_RADIUS before it was widened
local no_nav_since_ms = nil     -- when the plane last had neither GPS nor usable VIO (nil while it has one)
local rtl_decided = false       -- the RTL decision has been taken during this GPS outage
local weaving = false           -- flying home in S-turns (GUIDED) - VSW_WEAVE
local weave_origin, weave_bearing, weave_k, weave_target = nil, 0, 0, nil
local verr = -1
local last_report_ms = 0
local warned_vio = false

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

-- wind estimate north, east in m/s, or nil if unknown. ArduPilot 4.8 has ahrs:get_wind() (nil when
-- there is no valid estimate); 4.7 and older only have ahrs:wind_estimate()
local function wind_ne()
    if ahrs.get_wind then
        local wind = ahrs:get_wind()
        if wind then
            return wind:x(), wind:y()
        end
        return nil, nil
    end
    local wind = ahrs:wind_estimate()
    return wind:x(), wind:y()
end

local function limit_roll()
    local limit = VSW_ROLL:get()
    if limit > 0 and roll_saved == nil and limit < ROLL_LIMIT:get() then
        roll_saved = ROLL_LIMIT:get()
        ROLL_LIMIT:set(limit)
        -- an RTL circle tighter than the plane can turn at this bank makes it wander far from home;
        -- the tightest turn is needed downwind, where the ground speed is airspeed + wind
        local wn, we = wind_ne()
        local wind_speed = wn and math.sqrt(wn * wn + we * we) or 0
        local speed = AIRSPEED_CRUISE:get() + wind_speed + 2
        local needed = speed * speed / (9.81 * math.tan(math.rad(limit)))
        local radius = RTL_RADIUS:get()
        if radius == 0 then
            radius = WP_LOITER_RAD:get()   -- RTL_RADIUS 0 means: use WP_LOITER_RAD
        end
        if math.abs(radius) < needed then
            radius_saved = RTL_RADIUS:get()
            RTL_RADIUS:set(radius < 0 and -needed or needed)   -- negative = counter-clockwise, keep it
        end
        gcs:send_text(SEV_INFO, string.format("VSW: bank limit %.0f deg, RTL circle %.0f m", limit,
                                              math.max(math.abs(radius), needed)))
    end
end

local function restore_roll()
    if roll_saved ~= nil then
        ROLL_LIMIT:set(roll_saved)
        roll_saved = nil
    end
    if radius_saved ~= nil then
        RTL_RADIUS:set(radius_saved)
        radius_saved = nil
    end
end

-- EKF ground velocity vs airspeed along the nose plus the remembered wind, in m/s (nil: cannot judge)
local function velocity_mismatch()
    local vel = ahrs:get_velocity_NED()
    local eas = ahrs:airspeed_EAS()
    if vel == nil or eas == nil or wind_n == nil then
        return nil
    end
    local tas = eas * ahrs:get_EAS2TAS()
    local yaw = ahrs:get_yaw_rad()
    local dn = vel:x() - (tas * math.cos(yaw) + wind_n)
    local de = vel:y() - (tas * math.sin(yaw) + wind_e)
    return math.sqrt(dn * dn + de * de)
end

-- time a check has been failing: when it started, or nil while it passes
local function failing_since(since_ms, failing, now_ms)
    if not failing then
        return nil
    end
    return since_ms or now_ms
end

local function go_vio()
    wind_n, wind_e = wind_ne()
    ahrs:set_posvelyaw_source_set(SRC_VIO)
    gcs:send_text(SEV_WARNING, "VSW: GPS lost, switched to visual odometry")
    state, innov_since_ms, speed_since_ms, warned_vio = ST_VIO, nil, nil, false
    limit_roll()
end

local function reject_vio(text)
    ahrs:set_posvelyaw_source_set(SRC_GPS)
    gcs:send_text(SEV_WARNING, text)
    gcs:send_text(SEV_WARNING, "VSW: dead reckoning until GPS returns")
    state, vio_rejected = ST_DR, true
    restore_roll()
end

local function go_gps(text)
    ahrs:set_posvelyaw_source_set(SRC_GPS)
    if text then
        gcs:send_text(SEV_WARNING, text)
    end
    state, vio_rejected, verr = ST_GPS, false, -1
    no_nav_since_ms, rtl_decided = nil, false
    restore_roll()
end

-- the k-th S-turn point: k x VSW_WEAVE_D along the line from where the weave began to home, VSW_WEAVE to the
-- right (odd k) or left (even k) of it
local function weave_point(k)
    local t = weave_origin:copy()
    t:offset_bearing(weave_bearing, k * VSW_WEAVE_D:get())
    t:offset_bearing(weave_bearing + ((k % 2 == 1) and 90 or -90), VSW_WEAVE:get())
    return t
end

-- instead of a straight RTL: GUIDED home in S-turns (false: not possible here - then plain RTL)
local function start_weave()
    local loc, home = ahrs:get_location(), ahrs:get_home()
    if VSW_WEAVE:get() <= 0 or loc == nil or home == nil or loc:get_distance(home) < 2 * VSW_WEAVE_D:get() then
        return false
    end
    if not vehicle:set_mode(MODE_GUIDED) then
        return false
    end
    weave_origin, weave_bearing, weave_k = loc, math.deg(loc:get_bearing(home)), 1
    weave_target = weave_point(weave_k)
    if not vehicle:set_target_location(weave_target) then
        vehicle:set_mode(MODE_RTL)
        return false
    end
    weaving = true
    return true
end

local function stop_weave(text)
    weaving = false
    vehicle:set_mode(MODE_RTL)
    gcs:send_text(SEV_WARNING, text)
end

-- once per update while weaving: next point, or hand over to RTL
local function update_weave()
    local loc, home = ahrs:get_location(), ahrs:get_home()
    if vehicle:get_mode() ~= MODE_GUIDED then
        weaving = false                          -- the pilot changed the mode: leave it to the pilot
    elseif state ~= ST_VIO or loc == nil or home == nil then
        stop_weave("VSW: weaving ended - RTL")   -- VIO rejected or GPS back
    elseif loc:get_distance(home) < 1.5 * VSW_WEAVE_D:get() then
        stop_weave("VSW: near home - RTL")
    elseif loc:get_distance(weave_target) < 0.3 * VSW_WEAVE_D:get()
            or loc:get_distance(home) < weave_target:get_distance(home) - 20 then
        weave_k = weave_k + 1                    -- reached (or passed) this point: on to the next
        weave_target = weave_point(weave_k)
        vehicle:set_target_location(weave_target)
    end
end

-- return home (RTL) once per GPS outage, if the plane is flying itself; why = start of the messages
local function decide_rtl(why)
    rtl_decided = true
    local mode = vehicle:get_mode()
    if mode == MODE_RTL then
        gcs:send_text(SEV_WARNING, "VSW: " .. why .. " - already in RTL")
    elseif not SELF_FLYING[mode] then
        gcs:send_text(SEV_WARNING, "VSW: " .. why .. " - pilot mode, no RTL")
    elseif vehicle:is_landing() or not vehicle:get_likely_flying() then
        gcs:send_text(SEV_WARNING, "VSW: " .. why .. " - landing, no RTL")
    elseif state == ST_VIO and start_weave() then
        gcs:send_text(SEV_WARNING, "VSW: " .. why .. " - home in S-turns")
    elseif vehicle:set_mode(MODE_RTL) then
        gcs:send_text(SEV_WARNING, "VSW: " .. why .. " - returning home (RTL)")
    else
        gcs:send_text(SEV_WARNING, "VSW: " .. why .. " - RTL refused")
    end
end

local function update()
    if VSW_ENABLE:get() < 1 then
        restore_roll()
        return update, 1000
    end
    local now_ms = millis():toint()

    if gps_usable(now_ms) then
        bad_since_ms = nil
        good_since_ms = good_since_ms or now_ms
    else
        good_since_ms = nil
        bad_since_ms = bad_since_ms or now_ms
    end
    local gps_lost = bad_since_ms and now_ms - bad_since_ms >= VSW_BAD_MS:get()
    local gps_back = good_since_ms and now_ms - good_since_ms >= VSW_GOOD_S:get() * 1000
    local vio_healthy = visual_odom:healthy()
    local ivr = ahrs:get_variances() or -1   -- velocity innovation test ratio of the active source

    if state ~= ST_GPS and not arming:is_armed() then
        go_gps(nil)
    elseif state == ST_GPS then
        if gps_lost and arming:is_armed() then
            if vio_healthy and not vio_rejected then
                go_vio()
                if VSW_GPS_RTL:get() > 0 and not rtl_decided then
                    decide_rtl("GPS lost, on VIO")
                end
            else
                gcs:send_text(SEV_WARNING, "VSW: GPS lost, no VIO - dead reckoning")
                state = ST_DR
            end
        end
    elseif state == ST_VIO then
        if gps_back then
            go_gps("VSW: GPS good again, switched back to GPS")
        else
            verr = velocity_mismatch() or -1
            local reject_ms = VSW_REJ_T:get() * 1000
            -- no fresh VIO data during a dropout, so the IMU check only runs while the VIO is healthy
            innov_since_ms = failing_since(innov_since_ms,
                vio_healthy and VSW_INNOV:get() > 0 and ivr > VSW_INNOV:get(), now_ms)
            speed_since_ms = failing_since(speed_since_ms, VSW_VEL_ERR:get() > 0 and verr > VSW_VEL_ERR:get(), now_ms)
            if innov_since_ms and now_ms - innov_since_ms >= reject_ms then
                reject_vio(string.format("VSW: VIO rejected, disagrees with IMU (%.1f)", ivr))
            elseif speed_since_ms and now_ms - speed_since_ms >= reject_ms then
                reject_vio(string.format("VSW: VIO rejected, %.0f m/s off airspeed", verr))
            elseif not vio_healthy and not warned_vio then
                gcs:send_text(SEV_WARNING, "VSW: visual odometry unhealthy while in use")
                warned_vio = true
            end
        end
    elseif state == ST_DR then
        if gps_back then
            go_gps("VSW: GPS good again")
        elseif vio_healthy and not vio_rejected then
            go_vio()
            if VSW_GPS_RTL:get() > 0 and not rtl_decided then
                decide_rtl("GPS lost, on VIO")
            end
        end
    end

    if weaving then
        update_weave()
    end

    no_nav_since_ms = failing_since(no_nav_since_ms, state == ST_DR or (state == ST_VIO and not vio_healthy), now_ms)
    if no_nav_since_ms and not rtl_decided and VSW_RTL_S:get() > 0
            and now_ms - no_nav_since_ms >= VSW_RTL_S:get() * 1000 then
        decide_rtl("no GPS or VIO")
    end

    local src = ahrs:get_posvelyaw_source_set() + 1
    local verr_now = state == ST_VIO and verr or -1
    logger:write("VSW", "St,Src,IVR,VErr,WN,WE,RLim,RRad", "BBffffff", state, src, ivr, verr_now, wind_n or 0,
                 wind_e or 0, ROLL_LIMIT:get(), RTL_RADIUS:get())
    if now_ms - last_report_ms >= 1000 then
        gcs:send_named_float("VSW_SRC", src)
        gcs:send_named_float("VSW_ST", state)
        gcs:send_named_float("VSW_IVR", ivr)
        gcs:send_named_float("VSW_VERR", verr_now)
        last_report_ms = now_ms
    end
    return update, UPDATE_MS
end

gcs:send_text(SEV_INFO, "VSW: GPS/visual odometry switch-over loaded")
return update()
