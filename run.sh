#! /usr/bin/bash
pip install --target mavproxy/ mavproxy
cd kos
if [[ $* == '--no-server' ]]
	then
		./cross-build-sim-offline.sh &
	else
		./cross-build-sim-online.sh &
fi
cd ../ardupilot
./run_in_terminal_window.sh ArduCopter sitl/bin/arducopter -S --model + --speedup 1 --slave 0 --serial5=tcp:5765:wait --serial6=tcp:5766:wait --defaults copter.parm --sim-address=127.0.0.1 -I0
cd ../mavproxy
python MAVProxy/mavproxy.py --out 172.28.64.1:14550 --out 172.28.64.1:14551 --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501