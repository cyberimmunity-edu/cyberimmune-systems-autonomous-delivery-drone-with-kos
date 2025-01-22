# Вопросы и ответы

## Вопрос 0: Какой пароль для выданного VirtualBox-образа?

Логин и пароль одинаковые т.е. `user`.

## Вопрос 1: APM Planner 2 не подключается к цифровому двойнику запущенному через `make offline` или `make online` (и аналоги).

Отключите firewall. В Ubuntu это можно сделать так:

```bash
sudo systemctl stop ufw
```

p.s. если вы хотите оставить включенным firewall, то нужно настроить разрешения для подключения к определённым портам:

```bash
sudo ufw allow 14550/udp
```

## Вопрос 2: Чем отличаются `make offline` и `make online`.

Запуск компонентов без СУПА - `make offline`
Запуск всех компонентов, включая СУПА - `make online`

Предполагается, что предварительно запущен APM Planner 2 из папки planner.

## Вопрос 3: Что такое `make e2e-offline` и `make e2e-online`

Так запускаются сквозные автоматические тесты.
Эти тесты:

- запускают все необходимые компоненты,
- загружают задание,
- запускают задание,
- проверяют выполнение задания
- и останавливают все компоненты.

При запуске этих тестов модуль безопасности собирается из исходников и данное тестирование упрощает проверку корректности его работы (не требуется каждый раз вручную задавать полётное задание и т.п.).

## Вопрос 4: `docker images` отображает много `none` занимающих много места, это нормально?

Docker при сборке создаёт образ т.н. слоями. Каждая из команд создаёт новый слой. Каждый из таких слоёв будет отображаться как `none`, однако размер будет показывать размер конечного образа.

Например, Dockerfile для симулятора выглядит так:

```Dockerfile
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND noninteractive
ENV PATH="${PATH}:/opt/KasperskyOS-Community-Edition-1.2.0.89_en.deb/toolchain/bin:/home/user/.local/bin"
RUN apt-get update && \
    apt install -y \
        net-tools \
        python3 \
        python3-dev \
        python3-pip \
        python3-serial \
        python3-opencv \
        python3-wxgtk4.0 \
        python3-matplotlib \
        python3-lxml \
        python3-pygame \
        sudo \
        mc \
        vim \
        curl \
        tar \
        expect \
        build-essential \
        device-tree-compiler \
        && adduser --disabled-password --gecos "" user \
        && echo 'user ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers

COPY ./KasperskyOS-Community-Edition-1.2.0.89_en.deb /tmp

RUN apt install /tmp/KasperskyOS-Community-Edition-1.2.0.89_en.deb --assume-yes
RUN rm /tmp/KasperskyOS-Community-Edition-1.2.0.89_en.deb \
    && echo '/opt/KasperskyOS-Community-Edition-1.2.0.89/toolchain/lib' >> /etc/ld.so.conf.d/KasperskyOS.conf \
    && echo '/opt/KasperskyOS-Community-Edition-1.2.0.89/toolchain/x86_64-pc-linux-gnu/aarch64-kos/lib/' >> /etc/ld.so.conf.d/KasperskyOS.conf \
    && ldconfig

RUN su -c 'pip3 install PyYAML mavproxy pymavlink --user --upgrade' user

COPY ./ardupilot /home/user/ardupilot
COPY ./kos /home/user/kos
COPY ./planner /home/user/planner
COPY ./tests /home/user/tests

RUN chown -R 1000:1000 /home/user

CMD ["bash"]
```

Если посмотреть конечный образ симулятора, то можно увидеть слои (в обратной последовательности):

```bash
→ docker history simulator
IMAGE          CREATED          CREATED BY                                      SIZE      COMMENT
c11168bc9996   10 minutes ago   CMD ["bash"]                                    0B        buildkit.dockerfile.v0
<missing>      10 minutes ago   RUN /bin/sh -c chown -R 1000:1000 /home/user…   400MB     buildkit.dockerfile.v0
<missing>      10 minutes ago   COPY ./tests /home/user/tests # buildkit        11.4kB    buildkit.dockerfile.v0
<missing>      10 minutes ago   COPY ./planner /home/user/planner # buildkit    55.6MB    buildkit.dockerfile.v0
<missing>      10 minutes ago   COPY ./kos /home/user/kos # buildkit            210kB     buildkit.dockerfile.v0
<missing>      10 minutes ago   COPY ./ardupilot /home/user/ardupilot # buil…   165MB     buildkit.dockerfile.v0
<missing>      10 minutes ago   RUN /bin/sh -c su -c 'pip3 install PyYAML ma…   179MB     buildkit.dockerfile.v0
<missing>      11 minutes ago   RUN /bin/sh -c rm /tmp/KasperskyOS-Community…   99.1kB    buildkit.dockerfile.v0
<missing>      11 minutes ago   RUN /bin/sh -c apt install /tmp/KasperskyOS-…   2.27GB    buildkit.dockerfile.v0
<missing>      19 minutes ago   COPY ./KasperskyOS-Community-Edition-1.2.0.8…   263MB     buildkit.dockerfile.v0
<missing>      22 minutes ago   RUN /bin/sh -c apt-get update &&     apt ins…   1.86GB    buildkit.dockerfile.v0
<missing>      22 minutes ago   ENV PATH=/usr/local/sbin:/usr/local/bin:/usr…   0B        buildkit.dockerfile.v0
<missing>      22 minutes ago   ENV DEBIAN_FRONTEND=noninteractive              0B        buildkit.dockerfile.v0
<missing>      13 months ago    /bin/sh -c #(nop)  CMD ["/bin/bash"]            0B
<missing>      13 months ago    /bin/sh -c #(nop) ADD file:140fb5108b4a2861b…   77.8MB
<missing>      13 months ago    /bin/sh -c #(nop)  LABEL org.opencontainers.…   0B
<missing>      13 months ago    /bin/sh -c #(nop)  LABEL org.opencontainers.…   0B
<missing>      13 months ago    /bin/sh -c #(nop)  ARG LAUNCHPAD_BUILD_ARCH     0B
<missing>      13 months ago    /bin/sh -c #(nop)  ARG RELEASE                  0B
```

Тут же можно увидеть и размер слоёв.

## Вопрос 5: При запуске `make online` мой код не работает, и не выводятся координаты. Значит файл вообще не запускается?

При запуске `make online` задание подтверждается через СУПА, а значит в коде нужно учитывать, получил ли контроллер задание. В ином случае логика может отработать до получения задания и ничего не вывести на экран.

## Вопрос 6: Цифровой двойник не запускается `make online` или `make offline`. Ошибки сети?

Если ошибки от docker или docker-compose:

```text
ERROR: Pool overlaps with other one on this address space
```

```text
ERROR: for orvd  Cannot start service orvd: network a8cff278b3a741b9b3ae96395e1f238eb3400f9ec371eb4690c79e9d43421e7d not found
```

и разные их вариации.

Можно попробовать очистить запущенные сети `make clean-network`.

Если не помогает, то посмотреть, какие сети созданы, и удалить лишние (сети с именами bridge, host, none не надо трогать):

```bash
docker network list
```

и

```bash
docker network rm имя_сети
```

## Вопрос 7: docker не может скачать базовый образ и не собирает мои образы. Что делать?

Если ошибка как ниже:

```bash
→ docker run ubuntu:22.04
Unable to find image 'ubuntu:22.04' locally
docker: Error response from daemon: pull access denied for ubuntu, repository does not exist or may require 'docker login': denied: <html><body><h1>403 Forbidden</h1>
Since Docker is a US company, we must comply with US export control regulations. In an effort to comply with these, we now block all IP addresses that are located in Cuba, Iran, North Korea, Republic of Crimea, Sudan, and Syria. If you are not in one of these cities, countries, or regions and are blocked, please reach out to https://hub.docker.com/support/contact/
</body></html>.
See 'docker run --help'.
```

То достаточно использовать зеркала, например, так:

```bash
docker run cr.yandex/mirror/ubuntu:22.04
```

В файлах для сборки Dockerfile FROM указывается аналогично.

```bash
FROM cr.yandex/mirror/ubuntu:22.04
```

## Вопрос 8: Как изменить ID дрона?

По умолчанию в качестве ID дрона в онлайн-сборке используется MAC-адрес интерфейса en0, в оффлайн-сборке - фиксированный ID "00:00:00:00:00:00". Задать пользовательский ID можно, добавив при запуске нужного cross-build-скрипта аргумент "--board-id пользовательский_id" (без кавычек). Другой вариант - заменить в нужном cross-build-скрипте параметр BOARD_ID="" на BOARD_ID="пользовательский_id".

## Вопрос 9: Как включить ветер в симуляторе?

В [документе с описанием инструментов](/docs/TOOLS.md#sitl-simulator) есть полезные материалы. Например, перейдя по ссылке о симуляторе можно найти документацию о [параметрах симуляции](https://ardupilot.org/plane/docs/parameters.html#parameters-sim).

Для симуляции ветра подходит параметр `SIM_WIND_SPD`.

Установить параметр можно при помощи MAVProxy.

```bash
set param set SIM_WIND_SPD 25
```

## Вопрос 10: Как узнавать точки полётного задания (миссии)?

Вспомогательная структура [MissionCommand](/kos/flight_controller/include/flight_controller.h) хранит все необходимые параметры полётного задания.

## Вопрос 11: В интерфейсе СУПА не работают кнопки Arm/Disarm, и при сборке make online в планнере точки плана не появляются. О чём это говорит?

Полезно посмотреть [архитектуру](ARCHITECTURE.md), и пересмотреть вводные видео работы решения, где можно выделить основные этапы запуска машинки.

Предполагается, что в запуске машинки участвуют как минимум две роли: оператор машинки и диспетчер системы управления парком автомобилей (СУПА).

Оператор:

- готовит машинку к запуску,
- задаёт задание машинке, используя наземную станцию (планировщик, MAVProxy, APM Planner 2 и т.п.),
- регистрирует задание в СУПА для конкретной машинке по его Drone ID, используя Mission Sender,
- запускает машинку, получив подтверждение от СУПА.

Диспетчер СУПА:

- получает запрос на начало исполнения конкретного задания,
- разрешает запуск двигателей,
- при необходимости останавливает исполнение миссии.

## Вопрос 12: В терминале на видео занятий некоторые файлы выделены зелёным, а у меня они выделены как красные. Плохо ли это?

Короткий ответ - нет. Если интересно, как работают цвета в терминале, то можно почитать про bash, например, тут: [Bash scripting guide](https://opennet.ru/docs/RUS/bash_scripting_guide/) или открыть документацию по команде ls (и переменной LS_COLORS): `man ls`.

## Вопрос 13: При исполнении `./run.sh` возникает ошибка, т.к. нет директории MAVProxy.

Перед запуском `./run.sh` нужно установить необходимые компоненты как это описано в документации - [Способ 2](/docs/DEVELOPMENT.md#способ-2-запуск-цифрового-двойника-в-ubuntu-2204-без-docker-контейнеров)

## Вопрос 14: Как автопилот считает прохождение точек, и как он вообще устроен?

Следует исходить из того, что автопилот -- это чёрный ящик, и его устройство неизвестно. С точки зрения архитектуры главной задачей является выполнение целей и предположений безопасности.

## Вопрос 15: При запуске сборки при помощи скрипта cross-build.sh у меня появляются странные ошибки?

Если SDK находится в правильном месте, и ошибки не вызваны его отсутствием, то можно попробовать удалить прошлые сборки в папках build*:

```bash
cd kos
rm -Rf build*
```

## Вопрос 16: Для блокировки сброса груза я отредактировал main.cpp в periphery_controller. Правильно ли подобное решение?

Для экспериментальных целей или отладки можно менять любую часть проекта, но корректнее реализовать блокировку в файле `kos/flight_controller`.
Весь требуемый функционал вызывается через специально подготовленное [API](/docs/API.md).

## Вопрос 17: Как проверить в симуляторе, что я преодолеваю киберпрепятствия?

Использовать симулятор sitl_obstacles вместо обычного sitl.
Можно запустить:

- docker-версия (способ 1)
  - online `make online-obstacles` и сквозной тест `make e2e-online-bstacles`
  - offline `make offline-obstacles` и сквозной тест `make e2e-offline-obstacles`
- хост-версия (способ 2)
  - online `./run.sh --with-obstacles`
  - offline `./run.sh --with-obstacles --no-server`

## Вопрос 18: Как настроить кэш для APM Planner 2?

В интерфейсе APM Planner 2 выделить область на карте, используя ALT или SHIFT, и внизу нажать Cache.

## Вопрос 19: Как убрать лишние папки в .gitignore, чтобы они не попадали в коммиты и код?

В репозитории хранить желательно только само решение и вспомогательные файлы. При запуске цифрового двойнике создаётся множество файлов сторонних программ, не имеющих отношения к решению. Эти файлы нужно исключать из репозитория и не отслеживать изменения, связанные с ними. В корне репозитория есть файл [.gitignore](/.gitignore), который позволяет исключать ненужные папки и файлы. [Подробная документация по ссылке](https://git-scm.com/docs/gitignore).

## Вопрос 20: Как пользоваться docker-решением без интернета?

Предлагается разделить Dockerfile на два файла: обычный и базовый. В базовый включить всё необходимое, кроме файлов проекта. В обычном -- унаследоваться от базового.

Например:

- Файл simulator-base.Dockerfile

```Dockerfile
FROM cr.yandex/mirror/ubuntu:22.04

ENV DEBIAN_FRONTEND noninteractive
ENV PATH="${PATH}:/opt/KasperskyOS-Community-Edition-1.2.0.45/toolchain/bin:/home/user/.local/bin"
RUN apt-get update && \
    apt install -y \
        net-tools \
        python3 \
        python3-dev \
        python3-pip \
        python3-serial \
        python3-opencv \
        python3-wxgtk4.0 \
        python3-matplotlib \
        python3-lxml \
        python3-pygame \
        sudo \
        mc \
        vim \
        curl \
        tar \
        expect \
        build-essential \
        device-tree-compiler \
        parted \
        fdisk \
        dosfstools \
        && adduser --disabled-password --gecos "" user \
        && echo 'user ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers

COPY ./KasperskyOS-Community-Edition-1.2.0.45.zip /tmp

RUN unzip /tmp/KasperskyOS-Community-Edition-1.2.0.45 -d /opt \
    && rm /tmp/*.zip \
    && ln -s /opt/KasperskyOS-Community-Edition-1.2.0.45 /opt/KasperskyOS-Local-Edition \
    && echo '/opt/KasperskyOS-Community-Edition-1.2.0.45/toolchain/lib' >> /etc/ld.so.conf.d/KasperskyOS.conf \
    && echo '/opt/KasperskyOS-Community-Edition-1.2.0.45/toolchain/x86_64-pc-linux-gnu/aarch64-kos/lib/' >> /etc/ld.so.conf.d/KasperskyOS.conf \
    && ldconfig

RUN su -c 'pip3 install PyYAML mavproxy pymavlink --user --upgrade' user

CMD ["bash"]
```

и файл Dockerfile:

```Dockerfile
FROM simulator-base

COPY ./ardupilot /home/user/ardupilot
COPY ./kos /home/user/kos
COPY ./planner /home/user/planner
COPY ./tests /home/user/tests

RUN chown -R 1000:1000 /home/user

CMD ["bash"]
```

Изменения в Makefile (добавляется правило):

```Makefile
docker-image-simulator-base:
    docker build -f simulator-base.Dockerfile -t simulator-base ./
```

Таким образом добавляется зависимость, которая не указывается в зависимостях Makefile.
Чтобы этот способ работал, нужно собрать базовый образ при помощи `make docker-image-simulator-base`, а затем пользоваться всем остальным, как обычно.

Подобное можно сделать и для СУПА.

## Вопрос 21: Как сделать экспорт и импорт docker-образа?

На примере базового образа из прошлого вопроса сохранить образ в файл можно так:

```bash
docker save simulator-base -o simulator-base.tar
```

Загрузить из файла:

```bash
docker load -i simulator-base.tar
```

## Вопрос 22: Как обходить киберпрепятствия, используя sitl_obstacles ?

1. При помощи docker
   1. Запуск без СУПА `make offline-obstacles` (задание вручную задаётся)
   1. Запуск с СУПА `make online-obstacles` (задание задаётся вручную и в СУПА)
   1. Запуск автоматического сквозного теста без СУПА `make e2e-offline-obstacles`
   1. Запуск автоматического сквозного теста с СУПА `make e2e-online-obstacles`
1. Без docker используя `./run.sh`
   1. Запуск без СУПА `./run.sh --with-obstacles --no-server` (задание вручную задаётся)
   1. Запуск с СУПА `./run.sh --with-obstacles` (задание задаётся врунчюу и в СУПА)

## Вопрос 23: Как останавливать apache2 (СУПА) на хост-системе?

Apache2 на хост-системе запускается при помощи systemd, управление сервисами происходит при помощи systemctl. 
Таким образом можно:

- остановить apache2 (СУПА) `systemctl stop apache2`
- запустить apache2 (СУПА) `systemctl start apache2`
- посмотреть статус apache2 (СУПА) `systemctl status apache2`

## Вопрос 24: Ошибки QEMU (qemu-system-aarch64: Slirp: Failed to send packet, ret: -1)

Не мешают работе эмуляции и могут быть проигнорированы.

## Вопрос 25: Последовательность действий для запуска без СУПА (offline):

  1. Запустить `make offline` (аналогично и для `./run.sh --no-server`)
  2. Запустить APM Planner 2
  3. Дождаться, когда всё загрузилось и подключилось
  4. Загрузить задание в APM Planner 2 (Flight plan -> Edit waypoints -> Load WP -> Write)
  5. Нажать ARM (получить и получить разрешение)
  6. Нажать ARM (запустить двигатели)
  7. Включить режим Auto

## Вопрос 26: Последовательность действий для запуска с СУПА (online):

  1. Запустить `make online` (аналогично и для `./run.sh`)
  2. Запустить APM Planner 2
  3. Дождаться, когда всё загрузилось и подключилось
  4. Загрузить задание в APM Planner 2 (Flight plan -> Edit waypoints -> Load WP -> Write)
  5. Зайти в СУПА и посмотреть, подключился ли дрон и какой у него ID
  6. Загрузить задание в СУПА, используя Mission Sender
  7. В СУПА подтвердить миссию и разрешить взлёт
  8. В APM Planner 2 нажать ARM (запросить разрешение)
  9. В СУПА разрешить запуск двигателей
  10. В APM Planner 2 нажать ARM (запустить двигатели)
  11. Включить режим Auto

## Вопрос 27: Какой формат файла exampleMission.txt ?

Формат файла можно посмотреть в документации [Mavlink: File Formats](https://mavlink.io/en/file_formats/).
Однако, намного удобнее загрузить задание из файла в APM Planner 2, где все поля получат текстовую расшифровку.

## Вопрос 28: Что возвращает метод API метод getCoords() ?

API метод [getCoords()](./API.md#int-getcoordsint32_t-latitude-int32_t-longitude-int32_t-altitude) возвращает то, что ему приходит от GPS прёмника подключенного к модулю безопасности.

## Вопрос 29: В какой именно системе координат мы получаем данные от функции getCoords?

Подразумевается использование WGS 84.

## Вопрос 30: Мы ведь можем использовать какие-то стандартные библиотеки по типу vector?

Да. Что касается библиотек, можно посмотреть [документацию к SDK](https://support.kaspersky.ru/help/KCE/1.2/ru-RU/included_third_party_libs.htm) и найти версию поставляемого компилятора, в документации которого указаны поддерживаемые стандартные библиотеки (стандарт и т.п.).

## Вопрос 31: Какова требуемая точность позиционирования (насколько большое отклонение от задания еще считается допустимым) ?

Не более 5 метров. Коридор 10 метров.

## Вопрос 32: Гарантируется ли, что дрон перевозит лишь один груз?

Планируется перевозка только одного груза, однако не обязательно он будет загружен на первой точке.

## Вопрос 33: Что у меня не так установлено, что я никак не могу запустить проект 2 способом?

Проверьте, что установленны пакеты: device-tree-compiler, parted, fdisk, dosfstools (пакеты для Ubuntu 22.04, другие дистрибутивы по аналогии).

## Вопрос 34: Как изменить положение точки HOME у цифрового двойника?

Для цифрового двойника точка дома указывается при запуске симулятора SITL. При работе в Ubuntu без использования Docker дом можно поменять в скрипте run.sh (21 и 23 строчки, `--home=...`). При работе с использованием Docker дом задается в файлах docker-compose-offline-obstacles.yml, docker-compose-offline.yml, docker-compose-online-obstacles.yml, docker-compose-online.yml для каждого из сценариев (во всех файлах 25 строка, `--home=...`).

## Вопрос 35: На что влияет точка HOME, задаваемая в файле миссии?

В большинстве сценариев для автопилота эта точка не влияет ни на что. Однако именно она передается в модуль безопасности и отображается при вызове printMission(), поэтому предполагается, что ей нужно совпадать с настоящим положением дома.

## Вопрос 36: На что влияет точка HOME, задаваемая в файле server_connector_offline.cpp?

В оффлайн-режиме нет возможности получить миссию от сервера, поэтому используется фиксированная миссия, совпадающая с ardupilot/exampleMission.txt. Использование этой точки дома полностью совпадает с точкой дома, задаваемой в файле миссии.