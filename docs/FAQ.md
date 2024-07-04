# Вопросы и ответы

## Вопрос 0: Какой пароль для VirtualBox образа который выдавался?

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

Запуск компонентов без ОрВД - `make offline`
Запуск всех компонентов, включая ОрВД - `make online`

Предполагается, что предварительно запущен APM Planner 2 из папки planner.

## Вопрос 3: Что такое `make e2e-offline` и `make e2e-online`

Таким образом запускаются сквозные автоматические тесты.
Эти тесты:

- запускают все необходимые компоненты,
- загружают полётное задание,
- запускают полётное задание,
- проверяют выполнение полётного задания
- и останавливают все компоненты.

При запуске этих тестов модуль безопасности собирается из исходников и данное тестирование упрощает проверку корректности его работы (не требуется каждый раз вручную задавать полётное задание и т.п.).

## Вопрос 4: `docker images` отображает много `none` занимающих много места, это нормально?

Docker при сборке создаёт образ т.н. слоями. Каждая из команд создаёт новый слой. Каждый из таких слоёв будет отображаться как `none`, однако размер будет показывать размер конечного образа.

Например, наш Dockerfile для симулятора выглядит так:

```Dockerfile
FROM ubuntu:22.04

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

COPY ./ardupilot /home/user/ardupilot
COPY ./kos /home/user/kos
COPY ./planner /home/user/planner
COPY ./tests /home/user/tests

RUN chown -R 1000:1000 /home/user

CMD ["bash"]
```

А если посмотреть конечный образ симулятора, то увидим слои (в обратной последовательности):

```bash
→ docker history simulator
IMAGE          CREATED       CREATED BY                                      SIZE      COMMENT
2ed6960ffbb5   2 weeks ago   CMD ["bash"]                                    0B        buildkit.dockerfile.v0
<missing>      2 weeks ago   RUN /bin/sh -c chown -R 1000:1000 /home/user…   400MB     buildkit.dockerfile.v0
<missing>      2 weeks ago   COPY ./tests /home/user/tests # buildkit        7.54kB    buildkit.dockerfile.v0
<missing>      2 weeks ago   COPY ./planner /home/user/planner # buildkit    55.6MB    buildkit.dockerfile.v0
<missing>      2 weeks ago   COPY ./kos /home/user/kos # buildkit            178kB     buildkit.dockerfile.v0
<missing>      2 weeks ago   COPY ./ardupilot /home/user/ardupilot # buil…   166MB     buildkit.dockerfile.v0
<missing>      2 weeks ago   RUN /bin/sh -c su -c 'pip3 install PyYAML ma…   179MB     buildkit.dockerfile.v0
<missing>      2 weeks ago   RUN /bin/sh -c unzip /tmp/KasperskyOS-Commun…   1.89GB    buildkit.dockerfile.v0
<missing>      2 weeks ago   COPY ./KasperskyOS-Community-Edition-1.2.0.4…   511MB     buildkit.dockerfile.v0
<missing>      2 weeks ago   RUN /bin/sh -c apt-get update &&     apt ins…   1.83GB    buildkit.dockerfile.v0
<missing>      2 weeks ago   ENV PATH=/usr/local/sbin:/usr/local/bin:/usr…   0B        buildkit.dockerfile.v0
<missing>      2 weeks ago   ENV DEBIAN_FRONTEND=noninteractive              0B        buildkit.dockerfile.v0
<missing>      4 weeks ago   /bin/sh -c #(nop)  CMD ["/bin/bash"]            0B        
<missing>      4 weeks ago   /bin/sh -c #(nop) ADD file:a5d32dc2ab15ff0d7…   77.9MB    
<missing>      4 weeks ago   /bin/sh -c #(nop)  LABEL org.opencontainers.…   0B        
<missing>      4 weeks ago   /bin/sh -c #(nop)  LABEL org.opencontainers.…   0B        
<missing>      4 weeks ago   /bin/sh -c #(nop)  ARG LAUNCHPAD_BUILD_ARCH     0B        
<missing>      4 weeks ago   /bin/sh -c #(nop)  ARG RELEASE                  0B        
```

Тут же можно увидеть и размер слоёв.

## Вопрос 5: При запуске `make online` мой код не работает и не выводятся координаты (как в квалификационном задании) значит файл вообще не запускается?

При запуске `make online` полётное задание подтверждается через ОРвД, а значит в коде нужно учитывать получил ли полётный контроллер задание иначе логика может отработать до его получения и ничего не вывести на экран.

## Вопрос 6: Цифровой двойник не запускается `make online` или `make offline`. Ошибки сети?

Если ошибки от docker или docker-compose:

```text
ERROR: Pool overlaps with other one on this address space
```

```text
ERROR: for orvd  Cannot start service orvd: network a8cff278b3a741b9b3ae96395e1f238eb3400f9ec371eb4690c79e9d43421e7d not found
```

и разные вариации.

Можно попробовать очистить запущенные сети `make clean-network`.

Если не помогает, то посмотреть какие сети созданы и удалить лишние (сети с именами bridge, host, none не надо трогать):

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

То достаточно использовать зеркала, например так:

```bash
docker run cr.yandex/mirror/ubuntu:22.04
```

В файлах для сборки Dockerfile FROM указывается аналогично.

```bash
FROM cr.yandex/mirror/ubuntu:22.04
```

## Вопрос 8: Как изменить ID дрона?

ID дрона можно изменить в скриптах сборки модуля безопасности т.е. в kos/cross-build-*-*.sh переменная BOARD_ID (предустановленное значение 2).

## Вопрос 9: Как включить ветер в симуляторе?

В [документе с инструментарием](/docs/TOOLS.md#sitl-simulator) есть полезные материалы. Например перейдя по ссылке о симуляторе можно найти документацию о [параметрах симуляции](https://ardupilot.org/plane/docs/parameters.html#parameters-sim).

Для симуляции ветра подходит параметр `SIM_WIND_SPD`.

Установить параметр можно при помощи MAVProxy.

```bash
set param set SIM_WIND_SPD 25
```

## Вопрос 10: Как узнавать точки полётного задания (миссии)?

Вспомогательная структура [MissionCommand](/kos/flight_controller/include/mission.h) хранит все необходимые параметры полётного задания.

## Вопрос 11: В интерфейсе ОРВД не работают кнопки Arm/Disarm и при сборке make online в планнере точки плана не появляются. О чём это говорит?

Полезно посмотреть [архитектуру](ARCHITECTURE.md) и пересмотреть вводные видео работы решения где можно выделить основные этапы запуска квадрокоптера.

Исходя из архитектуры предполагается, что в запуске квадрокоптера участвуют как минимум две роли: оператор квадрокоптера и диспетчер системы организации воздушного движения (ОрВД).

Оператор:

- готовит квадрокоптер к запуску,
- задаёт полётное задание квадрокоптеру используя наземную станцию (планировщик, MAVProxy, APM Planner 2 и т.п.),
- регистрирует полётное задание в ОрВД для конкретного квадрокоптера по его Drone ID используя Mission Sender,
- запускает квадрокоптер получив подтверждение от ОрВД.

Диспетчер ОрВД:

- получает запрос на взлёт по конкретному полётному заданию,
- разрешает взлёт и запуск двигателей,
- при необходимости останавливает полёт.

## Вопрос 12: В терминале, на видео занятий, некоторые файлы выделены зелёным, а у меня они выделены как красные, плохо ли это?

Короткий ответ - нет. Если интересно как работают цвета в терминале, то можно почитать про bash, например, тут: [Bash scripting guide](https://opennet.ru/docs/RUS/bash_scripting_guide/) или открыть документацию по команде ls (и переменной LS_COLORS): `man ls`.

## Вопрос 13: При исполнении `./run.sh` ошибка т.к. нет директории MAVProxy

Перед запуском `./run.sh` нужно установить необходимые компоненты как это описано в документации - [Способ 2](/docs/DEVELOPMENT.md#способ-2-запуск-цифрового-двойника-в-ubuntu-2204-без-docker-контейнеров)

## Вопрос 14: А как автопилот считает прохождение точек и как, вообще, устроен автопилот?

Мы исходим из того, что автопилот это чёрный ящик и его устройство нам неизвестно. С точки зрения нашей архитектуры наша главная задача это выполнение целей и предположений безопасности.

## Вопрос 15: Запуская сборку при помощи ./cross-build*.sh скрипта у меня появляются странные ошибки?

Если SDK находит верно и ошибки дальше, то можно попробовать удалить прошлые сборки в папках build*:

```bash
cd kos
rm -Rf build*
```

## Вопрос 16: Для блокировки сброса груза я отредактировал main.cpp в periphery_controller правильно ли подобное решение?

Для эксперементальных целей или отладки вы можете менять любую часть проекта, но для соревнований реализация учитывается только внутри `kos/flight_controller`.
Весь недостающий требуемый функционал вызывается через специально подготовленное [API](/docs/API.md).

## Вопрос 17: А как убедиться в симуляторе, что киберпрепятствия я преодолеваю?

Использовать симулятор sitl_obstacles вместо обычного sitl.
Можно запустить:

- docker версия (способ 1)
  - online `make online-obstacles` и сквозной тест `make e2e-online-bstacles`
  - offline `make offline-obstacles` и сквозной тест `make e2e-offline-obstacles`
- хост версия (способ 2)
  - online `./run.sh --with-obstacles`
  - offline `./run.sh --with-obstacles --no-server`

## Вопрос 18: А как настроить кэш для APM Planner 2 ?

В интерфейсе APM Planner 2 выделить область на карте используя ALT или SHIFT и внизу нажать Cache

## Вопрос 19: Как убрать лишние папки в .gitignore, чтобы они не попадали в коммиты и код?

В репозитории хранить желательно только само решение и вспомогательные файлы. При запуске цифрового двойнике создаётся множество файлов сторонних программ не имеющих отношения к решению. Эти файлы нужно исключать из репозитория и не отслеживать изменения связанные с ними. В корне репозитория есть файл [.gitignore](/.gitignore) который позволяет исключать ненужные папки и файлы. [Подробная документация по ссылке](https://git-scm.com/docs/gitignore).

## Вопрос 20: Как пользоваться docker решением без интернета?

Предлагается разделить Dockerfile на два файла: обычный и базовый. В базовый включающий всё необходимое, но без файлов проекта. В обычном унаследоваться от базового.

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

Таким образом мы добавляем зависимость, но не указываем её в зависимостях Makefile.
Чтобы это работало, нам нужно собрать базовый образ при помощи `make docker-image-simulator-base` и пользоваться всем остальным как обычно.

Подобное можно сделать и для ОРВД.

## Вопрос 21: Как сделать экспорт и импорт docker образа?

На примере базового образа из прошлого вопроса сохранить образ в файл можно:

```bash
docker save simulator-base -o simulator-base.tar
```

Загрузить из файла:

```bash
docker load -i simulator-base.tar
```

## Вопрос 22: Как обходить выданные киберпрепятствия используя sitl_obstacles ?

1. При помощи docker
   1. Запуск без ОРВД `make offline-obstacles` (задание вручную задаётся)
   1. Запуск с ОРВД `make online-obstacles` (задание задаётся вручную и в ОРВД)
   1. Запуск автоматического сквозного теста без ОРВД `make e2e-offline-obstacles`
   1. Запуск автоматического сквозного теста с ОРВД `make e2e-online-obstacles`
1. Без docker используя `./run.sh`
   1. Запуск без ОРВД `./run.sh --with-obstacles --no-server` (задание вручную задаётся)
   1. Запуск с ОРВД `./run.sh --with-obstacles` (задание задаётся врунчюу и в ОРВД)

## Вопрос 23: Как останавливать apache2 (ОРВД) на хост системе?

Apache2 на хост системе запускается при помощи systemd и управление сервисами происходит при помощи systemctl. 
Таким образом можно:

- остановить apache2 (ОРВД) `systemctl stop apache2`
- запустить apache2 (ОРВД) `systemctl start apache2`
- посмотреть статус apache2 (ОРВД) `systemctl status apache2`

## Вопрос 24: Ошибки QEMU (qemu-system-aarch64: Slirp: Failed to send packet, ret: -1)

Не мешают работе эмуляции и не стоит обращать внимание.

## Вопрос 25: Последовательность действий для запуска без ОРВД (offline):

  1. Запустили `make offline` (аналогично и для `./run.sh --no-server`)
  2. Запустили APM Planner 2
  3. Дождались когда всё загрузилось и подключилось
  4. Загрузили полётное задание в APM Planner 2 (Flight plan -> Edit waypoints -> Load WP -> Write)
  5. Ткнули ARM (получили разрешение)
  6. Ткнули ARM (запустили двигатели)
  7. Включили режим Auto
  8. Запустили Mission start

## Вопрос 26: Последовательность действий для запуска с ОРВД (online):

  1. Запустили `make online` (аналогично и для `./run.sh`)
  2. Запустили APM Planner 2
  3. Дождались когда всё загрузилось и подключилось
  4. Загрузили полётное задание в APM Planner 2 (Flight plan -> Edit waypoints -> Load WP -> Write)
  5. Зашли в ОРВД и посмотрели подключился ли квадрокоптер и какой у него ID
  6. Загрузили полётное задание в ОРВД используя Mission Sender
  7. В ОРВД подтвердили миссию и разрешили взлёт
  8. В APM Planner 2 ткнули ARM (запросили разрешение)
  9. В ОРВД разрешили запуск двигателей
  10. В APM Planner 2 ткнули ARM (запустили двигатели)
  11. Включили режим Auto
  12. Запустили Mission start

## Вопрос 27: А какой формат файла exampleMission.txt ?

Формат файла можно посмотреть в документации [Mavlink: File Formats](https://mavlink.io/en/file_formats/).
Однако, намного удобнее загрузить полётное задание из файла в APM Planner 2 и все поля получат текстовую расшифровку.

## Вопрос 28: Что возвращает метод API метод getCoords() ?

API метод [getCoords()](./API.md#int-getcoordsint32_t-latitude-int32_t-longitude-int32_t-altitude) возвращает то, что ему приходит от GPS прёмника подключенного к модулю безопасности.

## Вопрос 29: В какой именно системе координат мы получаем данные от функции getCoords?

Подразумевается использование WGS 84.

## Вопрос 30: Мы ведь можем использовать какие-то стандартные библиотеки по типу vector?

Да. Что касается библиотек, можно посмотреть [документацию к SDK](https://support.kaspersky.ru/help/KCE/1.2/ru-RU/included_third_party_libs.htm) и найти версию поставляемого компилятора в документации которого указаны поддерживаемые стандартные библиотеки (стандарт и т.п.).

## Вопрос 31: Какова требуемая точность позиционирования (насколько большое отклонение от полетного задания еще считается допустимым) ?

Не более 5 метров. Коридор 10 метров.

## Вопрос 32: Может ли квадрокоптер в течение полетного задания совершить посадку и повторный взлет?

Только если это указано в полётном задании.

## Вопрос 33: Гарантируется ли, что точка приземления дрона (команда Land) находится под последней контрольной точкой (команда Waypoint) ?

Да.

## Вопрос 34: Гарантируется ли, что дрон перевозит лишь один груз?

Планируется перевозка только одного груза, однако не обязательно он будет загружен на первой точке.

## Вопрос 35: Что у меня не так установлено, что я никак не могу запустить по 2 способу?

Проверьте, что установленны пакеты: device-tree-compiler, parted, fdisk, dosfstools (пакеты для Ubuntu 22.04, другие дистрибутивы по аналогии).

## Вопросы 36: В ходе соревнований все 7 дней будут участвовать сразу все команды или каждой команде будет выделено по 1-2 дня?

У команд разный прогресс. Если участники не прошли киберпрепятствия в ЦД, то будут заниматься обходом киберпрепятствий в ЦД. Рекомендуется внимательно изучить [критерии оценки команд](./ASSESSMENT.md).

## Вопрос 37: Надо ли брать инструменты для сборки/разборки дрона?

Нет. Если дрон будет неисправен, то предполагается замена. (может, за исключением винтов, если винты сломаются, то выдадут новые).

## Вопрос 38: Разбор для транспортировки домой. Для этого потребуются инструменты?

Предполагаю, что нет. У ребят, собирающих квадрокоптеры должно быть всё необходимое.
