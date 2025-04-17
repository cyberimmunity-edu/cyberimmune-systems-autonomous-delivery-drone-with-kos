# Вопросы и ответы

## Вопрос 0: Какой пароль у выданного образа VirtualBox?

Логин и пароль одинаковые, т.е. `user`.

## Вопрос 1: APM Planner 2 не подключается к цифровому двойнику, запущенному через `make offline` или `make online` (и аналоги).

Отключите firewall. В Ubuntu это можно сделать так:

```bash
sudo systemctl stop ufw
```

P.S. Если вы хотите оставить включенным firewall, то нужно настроить разрешения для подключения к определённым портам:

```bash
sudo ufw allow 14550/udp
```

## Вопрос 2: Чем отличаются `make offline` и `make online`?

Запуск компонентов без ОрВД - `make offline`
Запуск всех компонентов, включая ОрВД - `make online`

Предполагается, что предварительно запущен APM Planner 2 из папки planner.

## Вопрос 3: Что такое `make e2e-offline` и `make e2e-online`?

Это команды для запуска сквозных автоматических тестов.
Эти тесты:

- запускают все необходимые компоненты,
- загружают полётное задание,
- запускают полётное задание,
- проверяют выполнение полётного задания
- останавливают все компоненты.

При запуске этих тестов модуль безопасности собирается из исходников и данное тестирование упрощает проверку корректности его работы (не требуется каждый раз вручную задавать полётное задание и т.п.).

## Вопрос 4: `docker images` отображает много `none`, занимающих много места. Это нормально?

Docker при сборке создаёт образ т.н. слоями. Каждая из команд создаёт новый слой. Каждый из таких слоёв будет отображаться как `none`, однако размер будет показывать размер конечного образа.

Например, Dockerfile для симулятора выглядит так:

```Dockerfile
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND noninteractive
ENV PATH="${PATH}:/opt/KasperskyOS-Community-Edition-wifi/toolchain/bin:/home/user/.local/bin"
RUN apt-get update && \
    apt upgrade -y && \
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
        jq \
        && adduser --disabled-password --gecos "" user \
        && echo 'user ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers

COPY ./KasperskyOS-Community-Edition-RaspberryPi4b-1.3.0_amd64.deb /tmp

RUN apt install /tmp/KasperskyOS-Community-Edition-RaspberryPi4b-1.3.0_amd64.deb -y
RUN rm /tmp/KasperskyOS-Community-Edition-RaspberryPi4b-1.3.0_amd64.deb \
    && echo '/opt/KasperskyOS-Community-Edition-RaspberryPi4b-wifi/toolchain/lib' >> /etc/ld.so.conf.d/KasperskyOS.conf \
    && echo '/opt/KasperskyOS-Community-Edition-RaspberryPi4b-wifi/toolchain/x86_64-pc-linux-gnu/aarch64-kos/lib/' >> /etc/ld.so.conf.d/KasperskyOS.conf \
    && ldconfig

RUN su -c 'pip3 install PyYAML mavproxy pymavlink --user --upgrade' user

COPY ./ardupilot /home/user/ardupilot
COPY ./kos /home/user/kos
COPY ./planner /home/user/planner
COPY ./tests /home/user/tests

RUN chown -R 1000:1000 /home/user

CMD ["bash"]
```

Если посмотреть на конечный образ симулятора, то можно увидеть его слои в обратной последовательности:

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

Тут же можно увидеть и размер слоев.

## Вопрос 5: При запуске `make online` мой код не работает, и не выводятся координаты (как в квалификационном задании). Это значит, что файл вообще не запускается?

При запуске `make online` полётное задание подтверждается через ОРвД, а значит в коде нужно учитывать, получил ли полётный контроллер задание. В инома случае логика может отработать до его получения, и ничего не будет выведено на экран.

## Вопрос 6: Цифровой двойник не запускается через `make online` или `make offline`. Ошибки сети?

Если ошибки от docker или docker-compose такие:

```text
ERROR: Pool overlaps with other one on this address space
```

```text
ERROR: for orvd  Cannot start service orvd: network a8cff278b3a741b9b3ae96395e1f238eb3400f9ec371eb4690c79e9d43421e7d not found
```

или разные их вариации, то можно попробовать очистить запущенные сети командой `make clean-network`.

Если не помогает, то посмотреть какие сети созданы, и удалить лишние (сети с именами bridge, host, none не надо трогать):

```bash
docker network list
```

и

```bash
docker network rm имя_сети
```

## Вопрос 7: Docker не может скачать базовый образ, и не собирает мои образы. Что делать?

Если ошибка следующего вида:

```bash
→ docker run ubuntu:22.04
Unable to find image 'ubuntu:22.04' locally
docker: Error response from daemon: pull access denied for ubuntu, repository does not exist or may require 'docker login': denied: <html><body><h1>403 Forbidden</h1>
Since Docker is a US company, we must comply with US export control regulations. In an effort to comply with these, we now block all IP addresses that are located in Cuba, Iran, North Korea, Republic of Crimea, Sudan, and Syria. If you are not in one of these cities, countries, or regions and are blocked, please reach out to https://hub.docker.com/support/contact/
</body></html>.
See 'docker run --help'.
```

то достаточно использовать зеркала:

```bash
docker run cr.yandex/mirror/ubuntu:22.04
```

В файлах для сборки Dockerfile FROM указывается аналогично.

```bash
FROM cr.yandex/mirror/ubuntu:22.04
```

## Вопрос 8: Как изменить ID дрона?

По умолчанию в качестве ID дрона в онлайн сборке используется MAC-адрес интерфейса en0, в оффлайн-сборке - фиксированный ID "00:00:00:00:00:00". Задать пользовательский ID можно, добавив при запуске нужного cross-build-скрипта аргумент "--board-id пользовательский_id" (без кавычек). Другой вариант - заменить в нужном cross-build-скрипте параметр BOARD_ID="" на BOARD_ID="пользовательский_id".

## Вопрос 9: Как включить ветер в симуляторе?

В [документе с инструментарием](/docs/TOOLS.md#sitl-simulator) есть полезные материалы. Например, перейдя по ссылке о симуляторе, можно найти документацию о [параметрах симуляции](https://ardupilot.org/plane/docs/parameters.html#parameters-sim).

Для симуляции ветра используется параметр `SIM_WIND_SPD`.

Установить параметр можно при помощи MAVProxy.

```bash
set param set SIM_WIND_SPD 25
```

## Вопрос 10: Как узнавать точки полётного задания (миссии)?

Вспомогательная структура [MissionCommand](/kos/flight_controller/include/flight_controller.h) хранит все необходимые параметры полётного задания.

## Вопрос 11: В интерфейсе ОРВД не работают кнопки Arm/Disarm, и при сборке make online в планнере точки плана не появляются. О чём это говорит?

Полезно посмотреть [архитектуру](ARCHITECTURE.md) и пересмотреть вводные видео работы решения, где можно выделить основные этапы запуска квадрокоптера.

Архитектура предполагает, что в запуске квадрокоптера участвуют двое: оператор квадрокоптера и диспетчер системы организации воздушного движения (ОрВД).

Оператор:

- готовит квадрокоптер к запуску,
- задаёт полётное задание квадрокоптеру, используя наземную станцию (планировщик, MAVProxy, APM Planner 2 и т.п.),
- регистрирует полётное задание в ОрВД для конкретного квадрокоптера по его Drone ID, используя Mission Sender,
- запускает квадрокоптер, получив подтверждение от ОрВД.

Диспетчер ОрВД:

- получает запрос на взлёт по конкретному полётному заданию,
- разрешает взлёт и запуск двигателей,
- при необходимости останавливает полёт.

## Вопрос 12: На видео занятий некоторые файлы в терминале выделены зелёным, а у меня - красным. Плохо ли это?

Короткий ответ - нет. Если интересно как работают цвета в терминале, то можно почитать про bash, например, тут: [Bash scripting guide](https://opennet.ru/docs/RUS/bash_scripting_guide/) или открыть документацию по команде ls (и переменной LS_COLORS): `man ls`.

## Вопрос 13: При исполнении `./run.sh` ошибка об остутсвии директории MAVProxy. Что делать?

Перед запуском `./run.sh` нужно установить необходимые компоненты как описано в документации - [Способ 2](/docs/DEVELOPMENT.md#способ-2-запуск-цифрового-двойника-в-ubuntu-2204-без-docker-контейнеров)

## Вопрос 14: Как автопилот считает прохождение точек? Как вообще устроен автопилот?

Нужно исходить из того, что автопилот - это чёрный ящик, и его устройство неизвестно. С точки зрения архитектуры главная задача модуля безопасности - выполнение целей и предположений безопасности.

## Вопрос 15: Когда я запускаю сборку при помощи ./cross-build.sh скрипта, у меня появляются странные ошибки.

Если SDK находится верно, а ошибки возникают дальше в процессе сборки, то можно попробовать удалить прошлые сборки в папке build:

```bash
cd kos
rm -Rf build*
```

## Вопрос 16: Для блокировки сброса груза я отредактировал main.cpp в periphery_controller. Правильно ли подобное решение?

Для экспериментальных целей или отладки можно менять любую часть проекта. Для соревнований реализация учитывается только внутри `kos/flight_controller`.
Весь недостающий требуемый функционал вызывается через специально подготовленное [API](/docs/API.md).

## Вопрос 17: Как убедиться в симуляторе, что я преодолеваю киберпрепятствия?

Использовать симулятор sitl с киберпрепятствиями вместо обычного sitl.
Его можно запустить так:

- docker версия (способ 1)
  - online `make online-obstacles` и сквозной тест `make e2e-online-bstacles`
  - offline `make offline-obstacles` и сквозной тест `make e2e-offline-obstacles`
- хост версия (способ 2)
  - online `./run.sh --with-obstacles`
  - offline `./run.sh --with-obstacles --no-server`

## Вопрос 18: Как настроить кэш для APM Planner 2?

В интерфейсе APM Planner 2 выделить область на карте, используя ALT или SHIFT, внизу нажать Cache.

## Вопрос 19: Как убрать лишние папки в .gitignore, чтобы они не попадали в коммиты и код?

В репозитории хранить желательно только само решение и вспомогательные файлы. При запуске цифрового двойнике создаётся множество файлов сторонних программ, не имеющих отношения к решению. Эти файлы нужно исключать из репозитория и не отслеживать изменения связанные с ними. В корне репозитория есть файл [.gitignore](/.gitignore), который позволяет исключать ненужные папки и файлы. [Подробная документация по ссылке](https://git-scm.com/docs/gitignore).

## Вопрос 20: Как пользоваться docker-решением без интернета?

Предлагается разделить Dockerfile на два файла: обычный и базовый. В базовый включить все зависимости без файлов проекта. Обычный унаследовать от базового.

Например:

- Файл simulator-base.Dockerfile

```Dockerfile
FROM cr.yandex/mirror/ubuntu:22.04

ENV DEBIAN_FRONTEND noninteractive
ENV PATH="${PATH}:/opt/KasperskyOS-Community-Edition-wifi/toolchain/bin:/home/user/.local/bin"
RUN apt-get update && \
    apt upgrade -y && \
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
        jq \
        && adduser --disabled-password --gecos "" user \
        && echo 'user ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers

COPY ./KasperskyOS-Community-Edition-RaspberryPi4b-1.3.0_amd64.deb /tmp

RUN apt install /tmp/KasperskyOS-Community-Edition-RaspberryPi4b-1.3.0_amd64.deb -y
RUN rm /tmp/KasperskyOS-Community-Edition-RaspberryPi4b-1.3.0_amd64.deb \
    && echo '/opt/KasperskyOS-Community-Edition-RaspberryPi4b-wifi/toolchain/lib' >> /etc/ld.so.conf.d/KasperskyOS.conf \
    && echo '/opt/KasperskyOS-Community-Edition-RaspberryPi4b-wifi/toolchain/x86_64-pc-linux-gnu/aarch64-kos/lib/' >> /etc/ld.so.conf.d/KasperskyOS.conf \
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

В Makefile добавиться правило:

```Makefile
docker-image-simulator-base:
    docker build -f simulator-base.Dockerfile -t simulator-base ./
```

Таким образом, зависимость добавлена, но она не указана в зависимостях Makefile.
Чтобы это решение работало, нужно собрать базовый образ при помощи `make docker-image-simulator-base`. Всем остальным пользоваться как обычно.

Подобное можно сделать и для ОрВД.

## Вопрос 21: Как сделать экспорт и импорт docker-образа?

Сохранение образа в файл на примере базового образа из прошлого вопроса:

```bash
docker save simulator-base -o simulator-base.tar
```

Загрузка из файла:

```bash
docker load -i simulator-base.tar
```

## Вопрос 22: Как обходить выданные киберпрепятствия, используя sitl_obstacles?

1. При помощи docker
   1. Запуск без ОРВД `make offline-obstacles` (задание задаётся вручную)
   2. Запуск с ОРВД `make online-obstacles` (задание задаётся вручную, в т.ч. и в ОРВД)
   3. Запуск автоматического сквозного теста без ОРВД `make e2e-offline-obstacles`
   4. Запуск автоматического сквозного теста с ОРВД `make e2e-online-obstacles`
2. Без docker, используя `./run.sh`
   1. Запуск без ОРВД `./run.sh --with-obstacles --no-server` (задание задаётся вручную)
   2. Запуск с ОРВД `./run.sh --with-obstacles` (задание задаётся вручную, в т.ч. и в ОРВД)

## Вопрос 23: Как останавливать apache2 (ОРВД) на хост-системе?

Apache2 на хост-системе запускается при помощи systemd. Управление сервисами происходит при помощи systemctl. 
Таким образом можно:

- остановить apache2 (ОРВД) `systemctl stop apache2`
- запустить apache2 (ОРВД) `systemctl start apache2`
- посмотреть статус apache2 (ОРВД) `systemctl status apache2`

## Вопрос 24: Ошибки QEMU (qemu-system-aarch64: Slirp: Failed to send packet, ret: -1)

Не мешают работе эмуляции, можно игнорировать.

## Вопрос 25: Последовательность действий для запуска без ОРВД (offline):

  1. Запустить `make offline` (аналогично для `./run.sh --no-server`)
  2. Запустить APM Planner 2
  3. Дождаться, когда всё загрузилось и подключилось
  4. Загрузить полётное задание в APM Planner 2 (Flight plan -> Edit waypoints -> Load WP -> Write)
  5. Нажать ARM (получить разрешение)
  6. Нажать ARM (запустить двигатели)
  7. Нажать Mission start

## Вопрос 26: Последовательность действий для запуска с ОРВД (online):

  1. Запустить `make online` (аналогично для `./run.sh`)
  2. Запустить APM Planner 2
  3. Дождаться, когда всё загрузилось и подключилось
  4. Загрузить полётное задание в APM Planner 2 (Flight plan -> Edit waypoints -> Load WP -> Write)
  5. Зайти в ОрВД и посмотреть, что квадрокоптер подключился, и узнать его ID
  6. Загрузить полётное задание в ОрВД, используя Mission Sender
  7. В ОрВД подтвердить миссию и разрешить взлёт
  8. В APM Planner 2 нажать ARM (запросить разрешение)
  9. В ОРВД разрешить запуск двигателей
  10. В APM Planner 2 нажать ARM (запустить двигатели)
  11. Запустить Mission start

## Вопрос 27: Какой формат у файла exampleMission.txt?

Формат файла можно посмотреть в документации [Mavlink: File Formats](https://mavlink.io/en/file_formats/).
Однако, намного удобнее загрузить полётное задание из файла в APM Planner 2, тогда все поля получат текстовую расшифровку.

## Вопрос 28: Что возвращает метод API метод getCoords()?

API-метод [getCoords()](./API.md#int-getcoordsint32_t-latitude-int32_t-longitude-int32_t-altitude) возвращает то, что ему приходит от GNSS-приемника, подключенного к модулю безопасности.

## Вопрос 29: В какой именно системе координат мы получаем данные от функции getCoords?

Подразумевается использование WGS 84.

## Вопрос 30: Мы ведь можем использовать какие-то стандартные библиотеки по типу vector?

Да. Что касается библиотек, можно посмотреть [документацию к SDK](https://support.kaspersky.ru/help/KCE/1.2/ru-RU/included_third_party_libs.htm), найти версию поставляемого компилятора, в документации которого указаны поддерживаемые стандартные библиотеки (стандарт и т.п.).

## Вопрос 31: Какова требуемая точность позиционирования (насколько большое отклонение от полетного задания еще считается допустимым) ?

Не более 5 метров. Коридор 10 метров.

## Вопрос 32: Может ли квадрокоптер в течение полетного задания совершить посадку и повторный взлет?

Только если это указано в полётном задании.

## Вопрос 33: Гарантируется ли, что точка приземления дрона (команда Land) находится под последней контрольной точкой (команда Waypoint) ?

Да.

## Вопрос 34: Гарантируется ли, что дрон перевозит лишь один груз?

Планируется перевозка только одного груза, однако не обязательно, что он будет загружен на первой точке.

## Вопрос 35: Что у меня не так установлено, что я никак не могу запустить 2 способом?

Проверьте, что установленны пакеты: device-tree-compiler, parted, fdisk, dosfstools (пакеты для Ubuntu 22.04, другие дистрибутивы по аналогии).

## Вопросы 36: В ходе соревнований все 7 дней будут участвовать сразу все команды или каждой команде будет выделено по 1-2 дня?

У команд разный прогресс. Если участники не прошли киберпрепятствия в ЦД, то будут заниматься обходом киберпрепятствий в ЦД. Рекомендуется внимательно изучить [критерии оценки команд](./ASSESSMENT.md).

## Вопрос 37: Надо ли брать инструменты для сборки/разборки дрона?

Нет. Если дрон будет неисправен, то предполагается замена, за исключением винтов: если винты сломаются, то выдадут новые.

## Вопрос 38: Разбор для транспортировки домой. Для этого потребуются инструменты?

Предполагается, что нет. У организаторов будет все необходимое.

## Вопрос 39: Как изменить положение точки HOME в цифровом двойнике?

Для цифрового двойника точка дома указывается при запуске симулятора SITL. При работе в Ubuntu без использования Docker дом можно поменять в скрипте run.sh (21 и 23 строчки, `--home=...`). При работе с использованием Docker дом задается в файлах docker-compose-offline-obstacles.yml, docker-compose-offline.yml, docker-compose-online-obstacles.yml, docker-compose-online.yml для каждого из сценариев (во всех файлах 25 строка, `--home=...`).

## Вопрос 40: На что влияет точка HOME, задаваемая в файле миссии?

В сценарии, предполагаемом соревнованиями, для автопилота эта точка не влияет ни на что. Однако именно она передается в модуль безопасности и отображается при вызове printMission(). Поэтому предполагается, что ей нужно совпадать с настоящим положением дома.

## Вопрос 41: На что влияет точка HOME, задаваемая в файле server_connector_offline.cpp?

В оффлайн-режиме нет возможности получить миссию от сервера, поэтому используется фиксированная миссия, совпадающая с ardupilot/exampleMission.txt. Использование этой точки дома полностью совпадает с точкой дома, задаваемой в файле миссии.