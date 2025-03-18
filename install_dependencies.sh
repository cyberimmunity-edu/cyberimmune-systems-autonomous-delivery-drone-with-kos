#! /usr/bin/bash
sudo add-apt-repository universe -y && sudo apt update
sudo apt upgrade
sudo apt install tmux unzip python3 python3-pip libfuse2 ./KasperskyOS-Community-Edition-1.2.0.89_en.deb
sudo pip3 install pyserial mavproxy
pip install --target mavproxy/ mavproxy
sudo chmod -R 777 mavproxy