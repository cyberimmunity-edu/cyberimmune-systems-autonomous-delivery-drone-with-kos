# Инструменты, используемые в проекте, и документация к ним

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

В цифровом двойнике используется версия KasperskyOS Community Edition SDK 1.3.0-wifi для RaspberryPi4b.
Наличие пакета с SDK требуется для запуска цифрового двойника, т.е. он необходим для прохождения квалификации и участия в соревнованиях.
Пакет SDK представляет собой Debian-пакет с именем KasperskyOS-Community-Edition-RaspberryPi4b-1.3.0_amd64.deb, который размещается в корне проекта.

### QEMU - эмулятор вычислительной машины

[Документация QEMU](https://www.qemu.org/docs/master/)

Используется для запуска собранного образа KasperskyOS Community Edition под ARM процессор.

Происходит прозрачно для разработчика при сборке и запуске проекта через скрипт cross-build.sh

## Цифровой двойник

### GNU make

[Документация GNU make](https://www.gnu.org/software/make/manual/make.html)

GNU make и [Makefile](https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos/blob/main/Makefile) используются для автоматизации запуска цифрового двойника.

Например в [Makefile](https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos/blob/main/Makefile) описаны следующие правила:

```bash
docker: docker-image

docker-image: docker-image-simulator docker-image-orvd

docker-image-simulator:
    docker build ./ -t simulator

docker-image-orvd:
    docker build -f orvd.Dockerfile -t orvd ./
```

Теперь для сборки образов достаточно запустить `make docker`, что автоматически запустит:

- правила для `docker-image` из [Makefile](https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos/blob/main/Makefile)
  - которые выполнят `docker-image-simulator` и `docker-image-orvd`
  - а те запустят:
    - `docker build ./ -t simulator`
    - и `docker build -f orvd.Dockerfile -t orvd ./`

### Docker

[Docker CLI, Dockerfile, Compose CLI, Compose file specifications](https://docs.docker.com/reference/)

Docker позволяет создавать и запускать легковесные виртуальные машины, т.н. контейнеры. Легковесные они, поскольку не запускают каждый раз операционную систему, а используют ядро уже запущенной операционной системы.
Описание того, что нужно включить в образ, и позднее, что появится в контейнере, созданном по образу, находится в файле [Dockerfile](https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos/blob/main/Dockerfile).

Сборка docker-образов автоматизирована в [Makefile](https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos/blob/main/Makefile), но можно собирать образы и вручную.
Например:

- `docker build ./ -t simulator` создаст образ simulator, используя описания из файла [Dockerfile](https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos/blob/main/Dockerfile).
- `docker build -f orvd.Dockerfile -t orvd ./` создаст образ orvd, используя описания из файла [orvd.Dockerfile](https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos/blob/main/orvd.Dockerfile).

Когда образ готов, то можно запустить на его базе контейнер и работать уже в нём.
Например:

- `docker run --user user -it --rm simulator /bin/bash -i`
  - запустит контейнер и `/bin/bash -i` внутри контейнера
  - на базе образа `simulator`
  - пользователь окружения установлен как `user`
  - флаг `--rm` указывает, что контейнер удалится после выхода из него
  - `-it` параметр говорит, что контейнер запустится в интерактивном режиме (мы в него сразу зайдём)

Мы используем Docker для запуска симуляции в контейнерах, но запуск возможен и без контейнеров.

### Docker Compose

[Документация Docker Compose](https://docs.docker.com/compose/).

Используется для автоматизации запуска множества контейнеров. Настраивается при помощи файлов с раширением `yml` и содержит в имени `docker-compose`

Запуск docker-compose автоматизирован правилами, описанными в Makefile. Зачастую его не требуется запускать напрямую.
Например, для запуска всех компонентов достаточно использовать `make online`, что автоматически создаст требуемые docker-образы и запустит их в соответствии с правилами, описанными в [docker-compose-online.yml](https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos/blob/main/docker-compose-online.yml).

Docker Compose используется для автоматизации создания сети и запуска контейнеров с необходимыми компонентами цифрового двойника. Но компоненты можно запустить и отдельно, без контейнеров и Docker Compose.

### ArduPilot - открытая система ПО для автономных систем

[Документация по проекту ArduPilot](https://ardupilot.org/dev/index.html)

#### APM Planner 2.0

[APM Planner 2.0](https://ardupilot.org/planner2/) - управляющая наземная станция для автономных систем с графическим интерфейсом.

Используется оператором для задания миссии квадрокоптеру и контроля за выполнением миссии в графическом режиме.

Демонстрация цифрового двойника с APM Planner 2.0 планировщиком на видео (youtube):
[![Демонстрация цифрового двойника и имитатора ОрВД](https://img.youtube.com/vi/ytzJ13hsMwg/0.jpg)](https://www.youtube.com/watch?v=ytzJ13hsMwg&t=305)

#### MAVProxy

[MAVProxy](https://ardupilot.org/mavproxy/index.html) - управляющая наземная станция для автономных систем с текстовым интерфейсом.

Используется разработчиком для задания миссии квадрокоптеру и контроля за выполнением миссии в текстовом режиме. Позволяет задавать параметры среды, например силу ветра.

Пример загрузки миссии и запуска квадрокоптера:

```text
wp load /home/user/cyberimmune-systems-autonomous-delivery-drone-with-kos/ardupilot/exampleMission.txt
wp loop
arm throttle
mode auto
long MAV_CMD_MISSION_START # можно ввести просто код команды 300
```

Демонстрация цифрового двойника с MAVProxy планировщиком на видео (youtube):
[![Цифровой двойник с MAVProxy планировщиком](https://img.youtube.com/vi/-VbmFeQ1A-Q/0.jpg)](https://www.youtube.com/watch?v=-VbmFeQ1A-Q)

- [MAVLINK Common Message Set](https://mavlink.io/en/messages/common.html)
- [Full Parameter List of Copter stable V4.5.0](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.5.0.html)

#### SITL Simulator

[SITL Simulator](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html) - симулятор для запуска квадрокоптера в виртуальной среде.
