# Документ для разработчиков

Оглавление:

- [Описаны два способа запуска цифрового двойника](#описаны-два-способа-запуска-цифрового-двойника)
  - [Способ 1. Запуск цифрового двойника при помощи docker контейнеров и docker-compose](#способ-1-запуск-цифрового-двойника-при-помощи-docker-контейнеров-и-docker-compose)
    - [Пример установки пакетов и запуска цифрового двойника для Ubuntu 22.04 (название пакетов в других дистрибутивах по аналогии)](#пример-установки-пакетов-и-запуска-цифрового-двойника-для-ubuntu-2204-название-пакетов-в-других-дистрибутивах-по-аналогии)
    - [Запуск APM Planner 2](#запуск-apm-planner-2)
  - [Способ 2. Запуск цифрового двойника в Ubuntu 22.04 без docker контейнеров](#способ-2-запуск-цифрового-двойника-в-ubuntu-2204-без-docker-контейнеров)
    - [Развертывание HTTP-сервера СУПА](#развертывание-http-сервера-супа)

## Описаны два способа запуска цифрового двойника

### Способ 1. Запуск цифрового двойника при помощи docker контейнеров и docker-compose

Данный способ подходит для любых Linux систем (включая запущенные в VirtualBox или в Windows WSL2).
Все компоненты цифрового двойника запускаются в контейнере. Планировщик с графическим интерфейсом может запускается как на хост системе так и на другом компьютере имеющим доступ к данному по сети.

#### Пример установки пакетов и запуска цифрового двойника для Ubuntu 22.04 (название пакетов в других дистрибутивах по аналогии)

Установка пакетов

```bash
# Получение информации о свежих версиях пакетов для дистрибутива
sudo apt-get update

# Установка необходимых пакетов
sudo apt-get install -y git make docker-compose docker.io libfuse2

# Удаление ненужных пакетов
sudo apt-get remove modemmanager -y

# Добавления пользователя user в нужные группы
sudo usermod -aG sudo,docker,dialout user

# Клонирование репозитория с симулятором (вместо cyberimmunity-edu может быть ваш fork)
git clone https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos.git

# Размещение KasperskyOS CE SDK 1.2.0.89 в папке проекта (копия репозитория)
cp ~/Downloads/KasperskyOS-Community-Edition-1.2.0.89_en.deb cyberimmune-systems-autonomous-delivery-drone-with-kos/

# Размещение KasperskyOS CE SDK 1.2.0.45 в папке проекта (копия репозитория)
cp ~/Downloads/KasperskyOS-Community-Edition-1.2.0.45.zip cyberimmune-systems-autonomous-delivery-drone-with-kos/
```

Запуск цифрового двойника:

```bash
# Запуск контейнеров с СУПА, SITL симулятором, компонентом на KasperskyOS, планировщиком MAVProxy
cd cyberimmune-systems-autonomous-delivery-drone-with-kos
make online
```

#### Запуск APM Planner 2

В новом терминале или запустив по иконке:

```bash
cd planner/
./APM_Planner.AppImage 
```

### Способ 2. Запуск цифрового двойника в Ubuntu 22.04 без docker контейнеров

Данный способ подходит для запуска в Ubuntu 22.04. Все компоненты устанавливаются на хост систему.

1. Клонировать репозиторий [https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos]

```bash
git clone https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos.git
```

1. Перейти в скачанную папку

```bash
cd cyberimmune-systems-autonomous-delivery-drone-with-kos
```

1. Разместить KasperskyOS-Community-Edition-1.2.0.89_en.deb и KasperskyOS-Community-Edition-1.2.0.45.zip в скачанной папке

1. Установить необходимые для работы компоненты, запустив скрипт `install_dependencies.sh`

```bash
./install_dependencies.sh
```

1. Сборка и запуск осуществляется при помощи скрипта `run.sh`
    1. Если в доступной сети развернут и запущен сервер СУПА, то дополнительных аргументов при запуске скрипта не требуется

    ```bash
    ./run.sh
    ```

    1. Если доступ к серверу СУПА отсутствует, то можно запустить симулятор без него. В этом случае все обращения к серверу СУПА будут считаться успешными. Также стоит учесть, что в этом случае в KOS не будет загружена полетная миссия

    ```bash
    ./run.sh --no-server
    ```

1. В процессе исполнения скрипта run.sh последовательно:
    1. Будет запущен APM Planner
    1. KOS будет собрана и запущена в эмуляторе QEMU
    1. Ardupilot будет запущен при помощи симулятора SITL
    1. Будет запущен mavproxy для обращения к APM Planner

1. Запустить APM Planner. Дождаться сборки и запуска всех частей симулятора. К работе можно приступать после того, как APM Planner подключится к симулятору SITL и загрузит параметры
1. Загрузить полетную миссию в симулятор (можно использовать тестовую миссию 'ardupilot/exampleMission.txt')
1. Попытаться произвести Arm. Он не будет произведен до тех пор, пока СУПА не даст соотвествующего разрешения (в случае запуска с параметром --no-server разрешение будет дано автмоатически)
1. Произвести Arm повторно; запустить миссию, когда Arm будет произведен

#### Развертывание HTTP-сервера СУПА

Установка необходимых пакетов:

```bash
cd ./cyberimmune-systems-autonomous-delivery-drone-with-kos/orvd
sudo chmod +x install.sh update.sh restart.sh
sudo ./install.sh
```

Теперь сервер доступен по IP виртуальной машины. IP адрес машины можно получить через `ifconfig` (или `ip a`).

Обновить сервер можно выполнив команду `sudo ./update.sh`, перезапустить - командой `sudo ./restart.sh`.
