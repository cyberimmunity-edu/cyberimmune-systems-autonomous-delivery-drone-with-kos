#!/bin/bash
mosquitto -c /etc/mosquitto/conf.d/default.conf -d &
apachectl -D FOREGROUND