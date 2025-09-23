
import argparse
import threading
import time
import random
import sys
from typing import Optional, Tuple

from pymavlink import mavutil

# -----------------------
# Copter mode mapping (ArduPilot custom_mode values)
# https://ardupilot.org/copter/docs/parameters.html#flightmode
# Note: only key modes included; add more as needed
COPTER_MODES = {
    "STABILIZE": 0,
    "ACRO": 1,
    "ALT_HOLD": 2,
    "AUTO": 3,
    "GUIDED": 4,
    "LOITER": 5,
    "RTL": 6,
    "CIRCLE": 7,
    "LAND": 9,
    "DRIFT": 11,
    "SPORT": 13,
    "FLIP": 14,
    "AUTOTUNE": 15,
    "POSHOLD": 16,
    "BRAKE": 17,
    "THROW": 18,
    "AVOID_ADSB": 19,
    "GUIDED_NOGPS": 20,
    "SMART_RTL": 21,
    "FLOWHOLD": 22,
    "FOLLOW": 23,
    "ZIGZAG": 24,
    "SYSTEMID": 25,
    "AUTOROTATE": 26,
    "AUTO_RTL": 27,
}

# Landed states from MAV_LANDED_STATE
LANDED_STATES = {
    "ON_GROUND": 1,
    "IN_AIR": 2,
    "TAKEOFF": 3,
    "LANDING": 4,
}

def send_heartbeat(m, mode_name: str, armed: bool):
    mode = COPTER_MODES.get(mode_name.upper(), 3)
    base_mode = 0
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1 << 5
    if armed:
        MAV_MODE_FLAG_SAFETY_ARMED = 1 << 7
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED
    base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    m.mav.heartbeat_send(
        type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
        autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode=base_mode,
        custom_mode=mode,
        system_status=mavutil.mavlink.MAV_STATE_ACTIVE if armed else mavutil.mavlink.MAV_STATE_STANDBY,
    )

def send_extended_sys_state(m, landed_state: int):
    m.mav.extended_sys_state_send(
        vtol_state=0,
        landed_state=landed_state,
    )

def send_global_position_int(m, lat: float, lon: float, rel_alt_m: float, vx=0, vy=0, vz=0):
    m.mav.global_position_int_send(
        int(time.time()*1e3),
        int(lat * 1e7),
        int(lon * 1e7),
        int((rel_alt_m + 0) * 1000),  # alt AMSL mm (we fake AMSL ~= rel_alt here)
        int(rel_alt_m * 1000),        # relative alt mm
        vx, vy, vz,
        0  # hdg * 100 (0..35999)
    )

def send_statustext(m, text: str, severity=6):
    # severity 6 = INFO
    txt = text[:50]
    m.mav.statustext_send(severity, txt.encode('utf-8'))

def for_all(links, fn, *a, **kw):
    for lnk in links:
        if lnk is not None:
            fn(lnk, *a, **kw)

def parse_mode_sequence(seq: str):
    """
    Accepts strings like: AUTO:15,RTL:20,AUTO:10
    Returns list of (mode, duration_sec)
    """
    out = []
    if not seq:
        return out
    for token in seq.split(","):
        token = token.strip()
        if not token:
            continue
        if ":" in token:
            mode, dur = token.split(":", 1)
            try:
                dur = float(dur)
            except Exception as e:
                raise ValueError(f"Invalid duration in '{token}': {e}")
        else:
            mode, dur = token, 0
        mode = mode.strip().upper()
        if mode not in COPTER_MODES:
            raise ValueError(f"Unknown mode '{mode}' in sequence")
        out.append((mode, dur))
    return out

def interactive_mode_thread(state):
    """
    Read commands from stdin to change mode while running.
    Commands:
      mode <NAME>     # e.g., mode RTL
      arm             # arm motors
      disarm          # disarm motors
      quit / exit     # stop simulator
    """
    print("[interactive] Type commands:  mode <NAME> | arm | disarm | quit")
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        cmd = parts[0].lower()
        if cmd == "mode" and len(parts) >= 2:
            name = parts[1].upper()
            if name not in COPTER_MODES:
                print(f"[interactive] Unknown mode '{name}'. Known: {', '.join(COPTER_MODES.keys())}")
                continue
            state["mode"] = name
            print(f"[interactive] Mode changed to {name}")
        elif cmd == "arm":
            state["armed"] = True
            print("[interactive] Armed")
        elif cmd == "disarm":
            state["armed"] = False
            print("[interactive] Disarmed")
        elif cmd == "help":
            print("[interactive] Commands: mode <NAME> | arm | disarm | help | quit")
        elif cmd in ("quit", "exit"):
            state["running"] = False
            print("[interactive] Stopping simulator...")
            break
        else:
            print("[interactive] Unknown command")

def main(args):
    # Open links
    links = []
    if args.gcs:
        links.append(mavutil.mavlink_connection(args.gcs, autoreconnect=True, source_system=args.sysid, source_component=args.compid))
    if args.pi:
        links.append(mavutil.mavlink_connection(args.pi, autoreconnect=True, source_system=args.sysid, source_component=args.compid))

    # State
    state = {
        "mode": args.mode.upper(),
        "armed": args.armed,
        "running": True,
        "landed_state": 1 if not args.start_in_air else 2,  # ON_GROUND or IN_AIR
        "lat": args.home_lat,
        "lon": args.home_lon,
        "rel_alt": 0.0 if not args.start_in_air else args.cruise_alt,
    }

    # Optional interactive stdin
    if args.interactive:
        t = threading.Thread(target=interactive_mode_thread, args=(state,), daemon=True)
        t.start()

    # Optional scripted mode sequence
    sequence = parse_mode_sequence(args.mode_seq)

    # Simple phase timing
    hz = args.rate_hz
    dt = 1.0 / hz
    t0 = time.time()
    seq_idx = 0
    next_switch_at = t0 + (sequence[0][1] if sequence else 1e18)

    print(f"Simulator started. Initial mode={state['mode']} armed={state['armed']}")
    if sequence:
        print(f"Mode sequence: {sequence}")

    try:
        while state["running"]:
            now = time.time()

            # Sequence-driven mode changes
            if sequence and now >= next_switch_at:
                seq_idx = (seq_idx + 1) % len(sequence)
                state["mode"] = sequence[seq_idx][0]
                next_switch_at = now + (sequence[seq_idx][1] if sequence[seq_idx][1] > 0 else 1e18)
                for_all(links, send_statustext, f"MODE -> {state['mode']}")

            # Simple altitude & landed-state behavior
            if state["mode"] == "AUTO" and state["armed"]:
                if state["rel_alt"] < args.cruise_alt:
                    state["rel_alt"] += args.vspeed * dt
                    if state["rel_alt"] > args.cruise_alt:
                        state["rel_alt"] = args.cruise_alt
                state["landed_state"] = 2 if state["rel_alt"] > 0.5 else 3  # IN_AIR / TAKEOFF
            elif state["mode"] == "RTL" and state["armed"]:
                if state["rel_alt"] > 0.2:
                    state["rel_alt"] -= args.vspeed * dt
                    state["landed_state"] = 4  # LANDING
                else:
                    state["rel_alt"] = 0.0
                    state["landed_state"] = 1  # ON_GROUND
                    if args.auto_disarm and state["armed"]:
                        state["armed"] = False
                        for_all(links, send_statustext, "Auto disarm after RTL")
            else:
                # Hold
                state["landed_state"] = 2 if state["rel_alt"] > 0.5 else 1

            # Heartbeat
            for_all(links, send_heartbeat, state["mode"], state["armed"])
            # Extended Sys State
            for_all(links, send_extended_sys_state, state["landed_state"])
            # Global position
            for_all(links, send_global_position_int, state["lat"], state["lon"], state["rel_alt"])

            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        for l in links:
            try:
                l.close()
            except:
                pass
        print("Simulator stopped.")

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Lightweight MAVLink FC simulator with configurable Copter mode")
    ap.add_argument("--gcs", default="udpout:127.0.0.1:14550", help="GCS MAVLink endpoint (e.g., udpout:127.0.0.1:14550)")
    ap.add_argument("--pi", default=None, help="Pi/companion MAVLink endpoint (optional)")
    ap.add_argument("--sysid", type=int, default=1, help="MAVLink system id")
    ap.add_argument("--compid", type=int, default=1, help="MAVLink component id")
    ap.add_argument("--rate-hz", type=float, default=5.0, help="Send rate (Hz)")
    ap.add_argument("--home-lat", type=float, default=27.535369)
    ap.add_argument("--home-lon", type=float, default=-80.360334)
    ap.add_argument("--cruise-alt", type=float, default=15.0, help="Target relative altitude for AUTO climb / cruise (m)")
    ap.add_argument("--vspeed", type=float, default=1.5, help="Vertical speed (m/s) used by simple AUTO/RTL climb/descend")
    ap.add_argument("--mode", default="AUTO", help=f"Initial Copter mode ({', '.join(COPTER_MODES.keys())})")
    ap.add_argument("--armed", action="store_true", help="Start armed")
    ap.add_argument("--start-in-air", action="store_true", help="Start with rel_alt>0 and IN_AIR")
    ap.add_argument("--auto-disarm", action="store_true", help="Auto-disarm when RTL completes landing")
    ap.add_argument("--mode-seq", default="", help="Comma list of MODE:seconds, e.g. 'AUTO:10,RTL:15,AUTO:0' (0=hold)")
    ap.add_argument("--interactive", action="store_true", help="Enable interactive stdin commands to change mode/arm state")
    args = ap.parse_args()

    # Validate initial mode
    if args.mode.upper() not in COPTER_MODES:
        raise SystemExit(f"Unknown --mode '{args.mode}'. Valid: {', '.join(COPTER_MODES.keys())}")

    main(args)
