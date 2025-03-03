# Инструменты используемые в проекте и документация к ним

Оглавление:

- [KasperskyOS Community Edition SDK](#kasperskyos-community-edition-sdk)
  - [QEMU - эмулятор вычислительной машины](#qemu---эмулятор-вычислительной-машины)
- [Цифровой двойник](#цифровой-двойник)
  - [GNU make](#gnu-make)
  - [Docker](#docker)
  - [Docker Compose](#docker-compose)
  - [ArduPilot - открытая система ПО для автономных систем](#ardupilot---открытая-система-по-для-автономных-систем)
    - [APM Planner 2.0](#apm-planner-20)
    - [MAVProxy](#mavproxy)
    - [SITL Simulator](#sitl-simulator)
  
## KasperskyOS Community Edition SDK

[Документация KasperskyOS Community Edition SDK](https://support.kaspersky.ru/help/KCE/1.1/ru-RU/whats_new.htm) (на данный момент более свежая документация недоступна).

В цифровом двойнике используется версия KasperskyOS Community Edition SDK 1.2.0.89.
Пакет SDK версии 1.2.0.89 представляет собой Debian-пакет с именем KasperskyOS-Community-Edition-1.2.0.89_en.deb и размещается в корне проекта.

### QEMU - эмулятор вычислительной машины

[Документация QEMU](https://www.qemu.org/docs/master/)

Используется для запуска собранного образа KasperskyOS Community Edition под ARM-процессор.

Использование QEMU происходит прозрачно для разработчика при сборке и запуске проекта посредством скрипта cross-build.sh.

## Цифровой двойник

### GNU make

[Документация GNU make](https://www.gnu.org/software/make/manual/make.html)

GNU make и [Makefile](https://gitflic.ru/project/learning-cyberimmunity/cyberimmune-systems-autonomous-delivery-drone-with-kos/blob/?file=Makefile&branch=rover) используются для автомазитации запуска цифрового двойника.

Например, в [Makefile](https://gitflic.ru/project/learning-cyberimmunity/cyberimmune-systems-autonomous-delivery-drone-with-kos/blob/?file=Makefile&branch=rover) описаны следующие правила:

```bash
docker: docker-image

docker-image: docker-image-simulator docker-image-afcs

docker-image-simulator:
    docker build ./ -t simulator

docker-image-afcs:
    docker build -f afcs.Dockerfile -t afcs ./
```

Для сборки этих образов достаточно запустить `make docker`, что автоматически запустит:

- правила для `docker-image` из [Makefile](https://gitflic.ru/project/learning-cyberimmunity/cyberimmune-systems-autonomous-delivery-drone-with-kos/blob/?file=Makefile&branch=rover)
  - которые выполнят `docker-image-simulator` и `docker-image-afcs`
  - а те запустят:
    - `docker build ./ -t simulator`
    - и `docker build -f afcs.Dockerfile -t afcs ./`

### Docker

[Docker CLI, Dockerfile, Compose CLI, Compose file specifications](https://docs.docker.com/reference/)

Docker позволяет создавать и запускать легковесные виртуальные машины, т.н. контейнеры. Они считаются легковесными, потому что не запускают операционную систему каждый раз, а используют ядро уже запущенной операционной системы.
Описание того, что нужно включить в образ, и того, что появится в контейнере созданном по образу, находится в файле [Dockerfile](https://gitflic.ru/project/learning-cyberimmunity/cyberimmune-systems-autonomous-delivery-drone-with-kos/blob?file=Dockerfile&branch=rover).

Сборка docker образов автоматизирована в [Makefile](https://gitflic.ru/project/learning-cyberimmunity/cyberimmune-systems-autonomous-delivery-drone-with-kos/blob/?file=Makefile&branch=rover), но можно собирать образы и вручную.
Например:

- `docker build ./ -t simulator` создаст образ simulator, используя описание из файла [Dockerfile](https://gitflic.ru/project/learning-cyberimmunity/cyberimmune-systems-autonomous-delivery-drone-with-kos/blob?file=Dockerfile&branch=rover).
- `docker build -f afcs.Dockerfile -t afcs ./` создаст образ afcs, используя описания из файла [afcs.Dockerfile](https://gitflic.ru/project/learning-cyberimmunity/cyberimmune-systems-autonomous-delivery-drone-with-kos/blob?file=afcs.Dockerfile&branch=rover).

Когда образ готов, можно запустить на его базе контейнер и работать уже в нём.
Например:

- `docker run --user user -it --rm simulator /bin/bash -i`
  - запустит контейнер и `/bin/bash -i` внутри контейнера
  - базой образа будет `simulator`
  - пользователь окружения будет установлен как `user`
  - флаг `--rm` указывает, что контейнер будет удален после выхода из него
  - `-it` параметр запустит контейнер в интерактивном режиме

Docker используется для запуска симуляции в контейнерах, но запуск возможен и без них.

### Docker Compose

[Документация Docker Compose](https://docs.docker.com/compose/).

Используется для автоматизации запуска множества контейнеров. Настраивается при помощи файлов с раширением `yml` и содержит в имени `docker-compose`

Запуск docker-compose автоматизирован правилами описанными в Makefile. Зачастую его не требуется запускать напрямую.
Например, для запуска всех компонентов достаточно вызвать команду `make online` из папки с проектом, что автоматически создаст требуемые Docker-образы и запустит их в соответствии с правилами, описанными в [docker-compose-online.yml](https://gitflic.ru/project/learning-cyberimmunity/cyberimmune-systems-autonomous-delivery-drone-with-kos/blob?file=docker-compose-online.yml&branch=rover).

Docker Compose используется для автоматизации создания сети и запуска контейнеров с необходимыми компонентами цифрового двойника. Эти компоненты можно запустить и отдельно, без контейнеров и Docker Compose.

### ArduPilot - открытая система ПО для автономных систем

[Документация по проекту ArduPilot](https://ardupilot.org/dev/index.html)

#### APM Planner 2.0

[APM Planner 2.0](https://ardupilot.org/planner2/) - управляющая наземная станция для автономных систем с графическим интерфейсом.

Используется оператором для задания миссии дрону и контроля за выполнением миссии в графическом режиме.

Демонстрация цифрового двойника (на примере квадрокоптера) с планировщиком APM Planner 2.0 на видео (youtube):
[![Демонстрация цифрового двойника и имитатора СУПА](https://img.youtube.com/vi/ytzJ13hsMwg/0.jpg)](https://www.youtube.com/watch?v=ytzJ13hsMwg&t=305)

#### MAVProxy

[MAVProxy](https://ardupilot.org/mavproxy/index.html) - управляющая наземная станция для автономных систем с текстовым интерфейсом.

Используется разработчиком для задания миссии дрону и контроля за выполнением миссии в текстовом режиме. Позволяет задавать параметры среды, например силу ветра.

Пример загрузки миссии и запуска дрона:

```text
wp load /home/user/cyberimmune-systems-autonomous-delivery-drone-with-kos/ardupilot/exampleMission.txt
wp loop
arm throttle
mode auto
long MAV_CMD_MISSION_START # можно ввести просто код команды 300
```

Демонстрация цифрового двойника (на примере квадрокоптера) с планировщиком MAVProxy на видео (youtube):
[![Цифровой двойник с MAVProxy планировщиком](https://img.youtube.com/vi/-VbmFeQ1A-Q/0.jpg)](https://www.youtube.com/watch?v=-VbmFeQ1A-Q)

- [MAVLINK Common Message Set](https://mavlink.io/en/messages/common.html)
- [Full Parameter List of Copter stable V4.5.0](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.5.0.html)

#### SITL Simulator

[SITL Simulator](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html) - симулятор для запуска дрона в виртуальной среде.
