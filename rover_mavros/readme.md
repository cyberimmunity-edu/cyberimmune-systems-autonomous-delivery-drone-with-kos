## Установка зависимостей

Добавить репозиторий ros:
```
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
```

Установить связку ros2+gz:
```
sudo apt install ros-humble-ros-gz ros-humble-desktop ros-dev-tools
sudo usermod -aG render,video $USER
```

Установить MAVROS:
```
sudo apt install ros-humble-mavros
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
chmod +x ./install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```

Прописать в .bashrc установку ros2 и перезапустить консоль:
```
source /opt/ros/humble/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
```

## Запуск нод

Сборка проекта:
```
rm -rf install/ build/ log/
colcon build
source install/setup.bash
```

Запуск вспомогательных нод (мосты до газебо, маврос, газебо):
```
source install/setup.bash
ros2 launch rover_mavros combined.launch.py
```

Запуск вспомогательных нод с камерой:

```
source install/setup.bash
ros2 launch rover_mavros combined.launch.py launch_camera:=true
```

Снимки камеры по умолчанию будут сохранены в captured_images.

Настроить параметры вспомогательных нод можно в `./src/rover_mavros/launch/`

Запуск основной управляющей ноды (на основе RC каналов из mavlink потока управляет ровером):
```
source install/setup.bash
ros2 run rover_mavros rover_controller
```

## Интеграция

1. Для тестирования можно запустить напрямую с ardupilot:
```
~/ardupilot/Tools/autotest/sim_vehicle.py -v Rover -f rover-skid --console --map --out 127.0.0.1:14571
```

2. Если СУПА развернут локально, то нужно удостовериться, что установлен pymavlink, и поменять значения переменных в afcs/utils/api_handlers:

`ENABLE_MAVLINK` сменить на `True`

`MAVLINK_CONNECTIONS_NUMBER` сменить на `1`

Если включен ufw, то прописать:

```
sudo ufw allow 14551/udp
sudo ufw allow 14571/udp
```

И перезапустить сервер с СУПА.

3. Если СУПА развернут в docker-контейнере, запускаемом через make online, то в ./src/rover_mavros/launch/mavros.launch.py изменить fcu_url на udp://:14551@localhost:14551 и пересобрать ноды через colcon.

## Проблемы

1. Иногда могут возникнуть проблемы с совместимостью QoS маврос ноды и управляющей ноды. Перезапуск управляющей ноды обычно помогает.