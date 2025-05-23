#! /usr/bin/bash
sudo add-apt-repository universe -y && sudo apt-get update
sudo apt-get upgrade
sudo apt-get install tmux unzip python3 python3-pip libfuse2 ./KasperskyOS-Community-Edition-RaspberryPi4b-1.3.0.166_ru.deb
sudo pip3 install pyserial mavproxy
pip install --target mavproxy/ mavproxy
sudo chmod -R 777 mavproxy