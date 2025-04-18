#! /usr/bin/bash
rm -f mavproxy/MAVProxy/mav.parm
rm -f mavproxy/MAVProxy/mav.tlog
rm -f mavproxy/MAVProxy/mav.tlog.raw
rm -f ardupilot/eeprom.bin
rm -rf ardupilot/logs
rm -rf ardupilot/terrain
tmux kill-session -t flight_controller
tmux new-session -d -s flight_controller
tmux send-keys -t flight_controller "cd planner; ./APM_Planner.AppImage" Enter
tmux split-window -h -t flight_controller
if [[ $* == *"--no-server"* ]]
	then
		tmux send-keys -t flight_controller "cd kos; ./cross-build.sh --target sim --mode offline" Enter
	else
		tmux send-keys -t flight_controller "cd kos; ./cross-build.sh --target sim --mode online" Enter
fi
tmux split-window -v -p 50 -t flight_controller
if [[ $* == *"--with-obstacles"* ]]
	then
		tmux send-keys -t flight_controller "cd ardupilot; ./run_in_terminal_window.sh ArduCopter sitl/arducopter_obstacles -S --model + --speedup 1 --slave 0 --serial5=tcp:5765:wait --serial6=tcp:5766:wait --serial7=tcp:5767:wait --defaults copter.parm --sim-address=127.0.0.1 --home=60.0026843,27.8573162,0.00,120 -I0" Enter
	else
		tmux send-keys -t flight_controller "cd ardupilot; ./run_in_terminal_window.sh ArduCopter sitl/arducopter -S --model + --speedup 1 --slave 0 --serial5=tcp:5765:wait --serial6=tcp:5766:wait --serial7=tcp:5767:wait --defaults copter.parm --sim-address=127.0.0.1 --home=60.0026843,27.8573162,0.00,120 -I0" Enter
fi
tmux select-pane -t flight_controller:0.0
tmux split-window -v -p 50 -t flight_controller
tmux send-keys -t flight_controller "cd mavproxy/MAVProxy; mavproxy.py --out='127.0.0.1:14550' --out='127.0.0.1:14551' --master='tcp:127.0.0.1:5760' --sitl='127.0.0.1:5501'" Enter
tmux select-pane -t flight_controller:0.0
tmux attach -t flight_controller