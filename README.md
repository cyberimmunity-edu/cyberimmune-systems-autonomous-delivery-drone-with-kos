#### Установки пакетов и запуска цифрового двойника для VirtualBox Ubuntu 22.04 

```bash
# Получение информации о свежих версиях пакетов для дистрибутива
sudo apt-get update

# Установка необходимых пакетов
sudo apt-get install -y git make docker-compose docker.io libfuse2

# Добавления пользователя user в нужные группы
sudo usermod -aG sudo,docker,dialout user

# Клонирование репозитория с симулятором (вместо cyberimmunity-edu может быть ваш fork)
git clone https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos.git

Запуск цифрового двойника:
```bash
# Запуск контейнеров с ОРВД, SITL симулятором, компонентом на KasperskyOS, планировщиком MAVProxy 
cd cyberimmune-systems-autonomous-delivery-drone-with-kos
make online
```

#### Запуск APM Planner 2

```bash
cd planner/
./APM_Planner.AppImage 
