# cyberimmune-systems-autonomous-delivery-drone-with-kos
Cyberimmune autonomous delivery drone prototype

## Запуск симулятора в Ubuntu
1. Установить необходимые для работы компоненты: python3, pip3 и библиотеки pyserial и mavproxy для python3; APM Planner
2. Клонировать репозиторий https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos
```
git clone https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos.git
```
3. Перейти в скачанную папку и изменить ветку на 'simulation'
```
cd cyberimmune-systems-autonomous-delivery-drone-with-kos
git checkout simulation
```
4. Сборка и запуск осуществляется при помощи скрипта 'run.sh'
    1. Если в доступной сети развернут и запущен сервер ОРВД, то дополнительных аргументов при запуске скрипта не требуется
    ```
    ./run.sh
    ```
    2. Если доступ к серверу ОРВД отсутствует, то можно запустить симулятор без него. В этом случае все обращения к серверу ОРВД будут считаться успешными. Также стоит учесть, что в этом случае в KOS не будет загружена полетная миссия
    ```
    ./run.sh --no-server
    ```
5. В процессе исполнения скрипта run.sh будет последовательно:
    1. Скачана библиотека mavproxy для python
    2. KOS будет собрана и запущена в эмуляторе QEMU
    3. Ardupilot будет запущен при помощи симулятора SITL
    4. Будет запущен mavproxy для обращения к APM Planner
6. Запустить APM Planner. Дождаться сборки и запуска всех частей симулятора. К работе можно приступать после того, как APM Planner подключится к симулятору SITL и загрузит параметры
7. Загрузить полетную миссию в симулятор (можно использовать тестовую миссию 'ardupilot/exampleMission.txt')
8. Попытаться произвести Arm. Он не будет произведен до тех пор, пока ОРВД не даст соотвествующего разрешения (в случае запуска с параметром --no-server разрешение будет дано автмоатически)
9. Произвести Arm повторно; запустить миссию, когда Arm будет произведен