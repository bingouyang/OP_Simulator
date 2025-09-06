import time, random
from pymavlink import mavutil

# ---------------- CONFIG ----------------
GCS_TARGET = 'udpout:127.0.0.1:14550'   # GCS on same PC
PI_TARGET  = 'udpout:<PI_IP>:14551'     # <-- replace with your Pi’s IP

SOURCE_SYS  = 1
SOURCE_COMP = 1
N_CYCLES    = 3   # number of mission cycles

HOME_LAT = 37.4000000
HOME_LON = -122.0500000

# (label, landed_state, duration_sec, lat, lon, rel_alt)
PHASES = [
    ("INIT_TAKEOFF",   mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,     5, HOME_LAT,    HOME_LON,    20),
    ("IN_AIR",         mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,     5, 37.4001000, -122.0501000, 30),
    ("LANDING",        mavutil.mavlink.MAV_LANDED_STATE_LANDING,    3, 37.4002000, -122.0502000, 10),
    ("SECOND_TAKEOFF", mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,     5, 37.4002000, -122.0502000, 25),
    ("FINAL_LANDING",  mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,  6, HOME_LAT,    HOME_LON,     0),
]

GPS_HZ = 5
HB_HZ  = 1

# ---------------- HELPERS ----------------
def make_out(target):
    return mavutil.mavlink_connection(target, source_system=SOURCE_SYS, source_component=SOURCE_COMP)

outs = [make_out(GCS_TARGET), make_out(PI_TARGET)]

def for_all(fn):
    for c in outs:
        try:
            fn(c)
        except Exception:
            pass

def send_heartbeat(c):
    c.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_QUADROTOR,
        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        0, 0,
        mavutil.mavlink.MAV_STATE_ACTIVE
    )

def send_extsys(c, landed_state):
    c.mav.extended_sys_state_send(
        mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED,
        landed_state
    )

def send_gpi(c, tboot_ms, lat_deg, lon_deg, rel_alt_m, vx=0, vy=0, vz=0, hdg_deg=0):
    lat = int(lat_deg * 1e7)
    lon = int(lon_deg * 1e7)
    alt_mm = int((50 + rel_alt_m) * 1000)
    rel_mm = int(rel_alt_m * 1000)
    hdg = int((hdg_deg % 360) * 100) if hdg_deg is not None else 65535
    c.mav.global_position_int_send(tboot_ms, lat, lon, alt_mm, rel_mm, vx, vy, vz, hdg)

def send_statustext(c, text, sev=mavutil.mavlink.MAV_SEVERITY_INFO):
    c.mav.statustext_send(sev, text.encode("utf-8")[:50])

# ---------------- MAIN ----------------
def main():
    print(f"Streaming to GCS={GCS_TARGET}, PI={PI_TARGET}")
    t0 = time.time()
    last_hb = 0.0
    hb_dt = 1.0 / HB_HZ

    for cycle in range(1, N_CYCLES + 1):
        print(f"\n=== Starting cycle {cycle}/{N_CYCLES} ===")
        for label, landed, dur, lat, lon, rel_alt in PHASES:
            # Special hook: announce "Mission complete" before FINAL_LANDING
            if label == "FINAL_LANDING":
                print(">>> Sending Mission complete STATUSTEXT")
                for_all(lambda c: send_statustext(c, "Mission complete"))

            print(f"---> {label}")
            for_all(lambda c: send_extsys(c, landed))

            start = time.time()
            gps_dt = 1.0 / GPS_HZ
            next_gps = 0.0

            while (time.time() - start) < dur:
                now = time.time()
                if (now - last_hb) >= hb_dt:
                    for_all(send_heartbeat)
                    last_hb = now
                if (now - start) >= next_gps:
                    tboot_ms = int((now - t0) * 1000)
                    for_all(lambda c: send_gpi(c, tboot_ms, lat, lon, rel_alt))
                    next_gps += gps_dt
                time.sleep(0.01)

            # random pause between phases
            pause = random.uniform(1, 3)
            print(f"--- dwell {pause:.1f}s before next phase ---")
            time.sleep(pause)

        print(f"=== Finished cycle {cycle}/{N_CYCLES} ===")

    print("All cycles complete.")

if __name__ == "__main__":
    main()
