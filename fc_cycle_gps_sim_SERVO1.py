import time, random
from pymavlink import mavutil

# ====================== CONFIG (Match Mission Planner) ======================
# Endpoints
GCS_TARGET = 'udpout:127.0.0.1:14550'    # GCS on same PC
PI_TARGET  = 'udpout:10.226.217.65:14551'# Pi’s IP

#PI_TARGET  = 'udpout:10.113.55.239:14551' # 
#PI_TARGET  = 'udpout:192.168.1.196:14551'# Pi’s IP
#PI_TARGET  = 'udpout:10.120.239.65:14551'# Pi’s IP
SOURCE_SYS  = 1
SOURCE_COMP = 1
N_CYCLES    = 2

# Simulator STATUSTEXT prefix and toggle
SIM_PREFIX = "SIMULATOR: "
ENABLE_SIM_STATUSTEXT = True  # set False to disable STATUSTEXT chatter

def sim_text(msg: str) -> str:
    return f"{SIM_PREFIX}{msg}"[:50]  # MAVLink STATUSTEXT max 50 bytes

def maybe_send_statustext(e, msg):
    if ENABLE_SIM_STATUSTEXT:
        send_statustext(e, sim_text(msg))

# Location
HOME_LAT = 27.535369
HOME_LON = -80.360334

# Mission Planner / CubeOrange servo mapping
SERVO_CH     = 9
PWM_ON       = 1900
PWM_OFF      = 1100
PWM_NEUTRAL  = 1500
SERVO_HZ     = 10

PWM_RELEASE  = 1800
PWM_RETRACT  = 1200

# Timing
GPS_HZ       = 5
HB_HZ        = 1
EXTSYS_HZ    = 2
MD_MIN       = 3
MD_MAX       = 5

PHASES = [
    ("INIT_TAKEOFF",   mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,     5, HOME_LAT,        HOME_LON,        20),
    ("IN_AIR",         mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,     15, HOME_LAT+0.0001, HOME_LON+0.0001, 30),
    ("LANDING",        mavutil.mavlink.MAV_LANDED_STATE_LANDING,    3, HOME_LAT+0.0002, HOME_LON+0.0002, 10),
    ("LANDED",         mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,  60, HOME_LAT+0.0004, HOME_LON+0.0002, 10),
    ("SECOND_TAKEOFF", mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,     5, HOME_LAT+0.0003, HOME_LON+0.0002, 25),
    ("IN_AIR2",        mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,     15, HOME_LAT+0.00045,HOME_LON+0.0002, 25),
    ("LANDING2",       mavutil.mavlink.MAV_LANDED_STATE_LANDING,    3, HOME_LAT+0.0005, HOME_LON+0.0002, 10),
    ("LANDED2",        mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,  60, HOME_LAT+0.0005, HOME_LON+0.0002, 10),
    ("THIRD_TAKEOFF",  mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,     5, HOME_LAT+0.0006, HOME_LON+0.0002, 25),
    ("FINAL_LANDING",  mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,  6, HOME_LAT,        HOME_LON,         0),
]

COPTER_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER",
    6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT", 13: "SPORT", 14: "FLIP",
    15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE", 18: "THROW", 19: "AVOID_ADSB",
    20: "GUIDED_NOGPS", 21: "SMART_RTL", 22: "FLOWHOLD", 23: "FOLLOW", 24: "ZIGZAG",
    25: "SYSTEMID", 26: "AUTOROTATE", 27: "AUTO_RTL",
}
ID_TO_MODE = {v: k for k, v in COPTER_MODES.items()}

def endpt_connect(target):
    return mavutil.mavlink_connection(target, source_system=SOURCE_SYS, source_component=SOURCE_COMP)

endpts = [endpt_connect(GCS_TARGET), endpt_connect(PI_TARGET)]

def update_all_endpts(fn):
    for endpt in endpts:
        try:
            fn(endpt)
        except Exception:
            pass

def send_heartbeat(endpt, armed, mode_name):
    mode = ID_TO_MODE.get(mode_name.upper(), 3)
    base_mode = 0
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1 << 5
    if armed:
        MAV_MODE_FLAG_SAFETY_ARMED = 1 << 7
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED
    base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    endpt.mav.heartbeat_send(
        type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
        autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode=base_mode,
        custom_mode=mode,
        system_status=mavutil.mavlink.MAV_STATE_ACTIVE if armed else mavutil.mavlink.MAV_STATE_STANDBY,
    )

def send_extsys(c, landed_state):
    c.mav.extended_sys_state_send(
        mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED,
        landed_state
    )

def send_gps(c, tboot_ms, lat_deg, lon_deg, rel_alt_m, vx=0, vy=0, vz=0, hdg_deg=0):
    lat = int(lat_deg * 1e7)
    lon = int(lon_deg * 1e7)
    alt_mm = int((50 + rel_alt_m) * 1000)
    rel_mm = int(rel_alt_m * 1000)
    hdg = int((hdg_deg % 360) * 100) if hdg_deg is not None else 65535
    c.mav.global_position_int_send(tboot_ms, lat, lon, alt_mm, rel_mm, vx, vy, vz, hdg)

def send_statustext(c, text, sev=mavutil.mavlink.MAV_SEVERITY_INFO):
    msg = f"{text}"[:50]
    c.mav.statustext_send(sev, msg.encode("utf-8"))

def _send_servo_robust(c, t_us, port, values8, chan_pwm):
    try:
        vals16 = [PWM_NEUTRAL]*16
        idx = max(1, min(16, SERVO_CH)) - 1
        vals16[idx] = chan_pwm
        c.mav.servo_output_raw_send(
            t_us, port,
            *vals16[:8], *vals16[8:16]
        )
    except TypeError:
        c.mav.servo_output_raw_send(t_us, port, *values8)

def send_servo_output(c, t_us, servo_on):
    target = PWM_ON if servo_on else PWM_OFF
    p0 = [PWM_NEUTRAL]*8
    p1 = [PWM_NEUTRAL]*8
    if 1 <= SERVO_CH <= 8:
        p0[SERVO_CH-1] = target
    elif 9 <= SERVO_CH <= 16:
        p1[SERVO_CH-9] = target
    _send_servo_robust(c, t_us, 0, p0, target)
    _send_servo_robust(c, t_us, 1, p1, target)

def main():
    print(f"Streaming to:\n  GCS={GCS_TARGET}\n  PI ={PI_TARGET}")
    print(f"[MP match] SERVO_CH={SERVO_CH}  ON={PWM_ON}  OFF={PWM_OFF}  Pi thresholds: RELEASE={PWM_RELEASE}, RETRACT={PWM_RETRACT}")
    t0 = time.time()
    last_hb = 0.0
    hb_dt   = 1.0 / HB_HZ
    armed   = True

    servo_on = False
    takeoff_off_deadline = None
    last_servo_tick = 0.0
    servo_period = 1.0 / SERVO_HZ

    for cycle in range(1, N_CYCLES + 1):
        print(f"\n=== Starting cycle {cycle}/{N_CYCLES} ===")
        did_disarm_announce = False

        for label, landed, dur, lat, lon, rel_alt in PHASES:
            if label == "LANDED" or label == "LANDED2":
                update_all_endpts(lambda e: maybe_send_statustext(e, "Landed"))
                update_all_endpts(lambda e: maybe_send_statustext(e, "Servo engaged"))
                print(">>> SERVO ON (after landing)")
                servo_on = True
                takeoff_off_deadline = None
            elif "TAKEOFF" in label:
                takeoff_off_deadline = time.time() + 2.0
                update_all_endpts(lambda e: maybe_send_statustext(e, "Takeoff detected"))
                print(">>> Takeoff detected — will SERVO OFF in 2s")

            if label == "FINAL_LANDING":
                update_all_endpts(lambda e: maybe_send_statustext(e, "Mission complete"))
                print(">>> Mission complete")

            print(f"---> {label}")
            start   = time.time()
            gps_dt  = 1.0 / GPS_HZ
            ext_dt  = 1.0 / EXTSYS_HZ
            next_gps = 0.0
            next_ext = 0.0

            while (time.time() - start) < dur:
                now = time.time()

                if (now - last_hb) >= hb_dt:
                    try:
                        with open('mode.txt', 'r') as f:
                            mode = f.read().strip() or "AUTO"
                    except Exception:
                        mode = "AUTO"
                    update_all_endpts(lambda e: send_heartbeat(e, armed, mode))
                    last_hb = now

                if (now - start) >= next_gps:
                    tboot_ms = int((now - t0) * 1000)
                    update_all_endpts(lambda e: send_gps(e, tboot_ms, lat, lon, rel_alt))
                    next_gps += gps_dt

                if (now - start) >= next_ext:
                    update_all_endpts(lambda e: send_extsys(e, landed))
                    next_ext += ext_dt

                if takeoff_off_deadline is not None and now >= takeoff_off_deadline:
                    servo_on = False
                    takeoff_off_deadline = None
                    update_all_endpts(lambda e: maybe_send_statustext(e, "Servo released"))
                    print(">>> SERVO OFF (2s post-takeoff)")

                if (now - last_servo_tick) >= servo_period:
                    t_us = int((now - t0) * 1e6)
                    update_all_endpts(lambda e: send_servo_output(e, t_us, servo_on))
                    last_servo_tick = now

                time.sleep(0.01)

            if label == "FINAL_LANDING" and not did_disarm_announce:
                armed = False
                update_all_endpts(lambda e: maybe_send_statustext(e, "Auto disarmed"))
                print(">>> Auto disarm")
                did_disarm_announce = True

            pause = random.uniform(MD_MIN, MD_MAX)
            print(f"--- dwell {pause:.1f}s ---")
            time.sleep(pause)

        print(f"=== Finished cycle {cycle}/{N_CYCLES} ===")
        armed = True

    print("All cycles complete.")

if __name__ == "__main__":
    main()
