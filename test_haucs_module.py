import traceback
try:
    import MAVProxy.modules.mavproxy_haucs
    print("haucs import OK: no missing imports")
except Exception:
    traceback.print_exc()