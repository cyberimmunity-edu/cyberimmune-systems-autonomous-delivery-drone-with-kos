# cyberimmune-systems-autonomous-delivery-drone-with-kos
Cyberimmune autonomous delivery drone prototype

## Запуск симулятора в Ubuntu
1. Клонировать репозиторий https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos
```
git clone https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos.git
```
2. Перейти в скачанную папку и изменить ветку на 'simulation'
```
cd cyberimmune-systems-autonomous-delivery-drone-with-kos
```
```
git checkout simulation
```
3. Установить необходимые для работы компоненты, запустив скрипт 'install_dependencies.sh'
```
./install_dependencies.sh
```
4. Скачать и установить deb-пакет KasperskyOS-Local-Edition
5. Сборка и запуск осуществляется при помощи скрипта 'run.sh'
    1. Если в доступной сети развернут и запущен сервер ОРВД, то дополнительных аргументов при запуске скрипта не требуется
    ```
    ./run.sh
    ```
    2. Если доступ к серверу ОРВД отсутствует, то можно запустить симулятор без него. В этом случае все обращения к серверу ОРВД будут считаться успешными. Также стоит учесть, что в этом случае в KOS не будет загружена полетная миссия
    ```
    ./run.sh --no-server
    ```
6. В процессе исполнения скрипта run.sh последовательно:
    1. Будет запущен APM Planner
    2. KOS будет собрана и запущена в эмуляторе QEMU
    3. Ardupilot будет запущен при помощи симулятора SITL
    4. Будет запущен mavproxy для обращения к APM Planner
7. Запустить APM Planner. Дождаться сборки и запуска всех частей симулятора. К работе можно приступать после того, как APM Planner подключится к симулятору SITL и загрузит параметры
8. Загрузить полетную миссию в симулятор (можно использовать тестовую миссию 'ardupilot/exampleMission.txt')
9. Попытаться произвести Arm. Он не будет произведен до тех пор, пока ОРВД не даст соотвествующего разрешения (в случае запуска с параметром --no-server разрешение будет дано автмоатически)
10. Произвести Arm повторно; запустить миссию, когда Arm будет произведен