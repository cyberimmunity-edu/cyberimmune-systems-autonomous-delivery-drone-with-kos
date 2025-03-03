#! /usr/bin/bash
sudo apt update
sudo apt upgrade
sudo apt install tmux
sudo apt install unzip
sudo apt install ./KasperskyOS-Community-Edition-1.2.0.89_en.deb
sudo apt install python3
sudo apt install python3-pip
sudo pip3 install pyserial
sudo pip3 install mavproxy
sudo add-apt-repository universe
sudo apt install libfuse2
pip install --target mavproxy/ mavproxy
sudo chmod -R 777 mavproxy