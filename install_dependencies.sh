#! /usr/bin/bash
sudo apt update
sudo apt upgrade
sudo apt install tmux
sudo apt install unzip
sudo apt install ./KasperskyOS-Community-Edition-1.2.0.89_en.deb
sudo rm -f /opt/KasperskyOS-Community-Edition-1.2.0.89/sysroot-aarch64-kos/bin/dnet_entity
sudo unzip -j KasperskyOS-Community-Edition-1.2.0.45.zip KasperskyOS-Community-Edition-1.2.0.45/sysroot-aarch64-kos/bin/dnet_entity -d /opt/KasperskyOS-Community-Edition-1.2.0.89/sysroot-aarch64-kos/bin
sudo apt install python3
sudo apt install python3-pip
sudo pip3 install pyserial
sudo pip3 install mavproxy
sudo add-apt-repository universe
sudo apt install libfuse2
pip install --target mavproxy/ mavproxy
sudo chmod -R 777 mavproxy