REM ================= FULL DUPLEX PORT MAP ======================
REM PC MAVProxy --master   udp:127.0.0.1:14550   :SIM/Drone
REM PC MAVProxy --out      udpin:0.0.0.0:14555   :from Pi
REM PC MAVProxy --out      udpout:<Pi_IP>:14551  :Pi 
REM PC MAVProxy --out      udp:127.0.0.1:14560   :Mission planner
REM =============================================================

REM python .\MAVProxy\mavproxy.py --master=udp:127.0.0.1:14550 --out=udpout:192.168.1.193:14551 --master=udpin:0.0.0.0:14555 --out=udp:127.0.0.1:14560 --aircraft=splashy --console --cmd "set moddebug 3"


cd ..

python setup.py build install --user

set MAVLINK_DIALECT=ardupilotmega
set MAVLINK20=1

python .\MAVProxy\mavproxy.py --master=udp:127.0.0.1:14550 --out=udpout:10.226.217.65:14551 --master=udpin:0.0.0.0:14555 --out=udp:127.0.0.1:14560 --aircraft=splashy --console --cmd "set moddebug 3"

REM python .\MAVProxy\mavproxy.py --master=udp:127.0.0.1:14550 --out=udpout:10.120.239.65:14551 --master=udpin:0.0.0.0:14555 --out=udp:127.0.0.1:14560 --aircraft=splashy --console --cmd "set moddebug 3"

REM python .\MAVProxy\mavproxy.py --master=udp:127.0.0.1:14550 --out=udpout:192.168.1.196:14551 --master=udpin:0.0.0.0:14555 --out=udp:127.0.0.1:14560 --aircraft=splashy --console --cmd "set moddebug 3"


pause