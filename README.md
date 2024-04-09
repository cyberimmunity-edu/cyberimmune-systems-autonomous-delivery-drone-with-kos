# cyberimmune-systems-autonomous-delivery-drone-with-kos

Cyberimmune autonomous delivery drone prototype

## Описаны два способа запуска симулятора

## Способ 1. Запуск симулятора при помощи docker контейнеров и docker-compose

### Пример установки пакетов и запуска симулятора для Ubuntu 22.04 (название пакетов в других дистрибутивах по аналогии)

 ```bash
sudo apt-get update
# Установка необходимых пакетов
sudo apt-get install -y git make docker-compose docker.io libfuse2
# Удаление ненужных пакетов
sudo apt-get remove modemmanager -y
# Добавления пользователя user в нужные группы
sudo usermod -aG sudo,docker,dialout user
# Клонирование репозитория с симулятором
git clone https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos.git
# Размещение KasperskyOS CE SDK 1.2 в репозитории
# cp KasperskyOS-Community-Edition-1.2.0.45.zip cyberimmune-systems-autonomous-delivery-drone-with-kos/
# Запуск контейнеров с ОРВД, SITL симулятором, компонентом на KasperskyOS, планировщиком MAVProxy 
cd cyberimmune-systems-autonomous-delivery-drone-with-kos
make online
```

### Запуск APM Planner 2

```bash
cd planner/
./APM_Planner.AppImage 
```

## Способ 2. Запуск симулятора в Ubuntu без docker контейнеров

1. Клонировать репозиторий [https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos]

```bash
git clone https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos.git
```

1. Перейти в скачанную папку и изменить ветку на 'simulation'

```bash
cd cyberimmune-systems-autonomous-delivery-drone-with-kos
```

1. Установить необходимые для работы компоненты, запустив скрипт 'install_dependencies.sh'

```bash
./install_dependencies.sh
```

1. Скачать и установить deb-пакет KasperskyOS-Local-Edition

1. Сборка и запуск осуществляется при помощи скрипта 'run.sh'
    1. Если в доступной сети развернут и запущен сервер ОРВД, то дополнительных аргументов при запуске скрипта не требуется

    ```bash
    ./run.sh
    ```

    1. Если доступ к серверу ОРВД отсутствует, то можно запустить симулятор без него. В этом случае все обращения к серверу ОРВД будут считаться успешными. Также стоит учесть, что в этом случае в KOS не будет загружена полетная миссия

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
1. Попытаться произвести Arm. Он не будет произведен до тех пор, пока ОРВД не даст соотвествующего разрешения (в случае запуска с параметром --no-server разрешение будет дано автмоатически)
1. Произвести Arm повторно; запустить миссию, когда Arm будет произведен
