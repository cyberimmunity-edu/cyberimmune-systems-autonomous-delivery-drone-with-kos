#### становки пакетов и запуска цифрового двойника для Ubuntu 22.04 (название пакетов в других дистрибутивах по аналогии)

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

# Размещение KasperskyOS CE SDK 1.2 в папке проекта (копия репозитория)
cp ~/Downloads/KasperskyOS-Community-Edition-1.2.0.45.zip cyberimmune-systems-autonomous-delivery-drone-with-kos/
```

Запуск цифрового двойника:

```bash
# Запуск контейнеров с ОРВД, SITL симулятором, компонентом на KasperskyOS, планировщиком MAVProxy 
cd cyberimmune-systems-autonomous-delivery-drone-with-kos
make online
```

#### Запуск APM Planner 2

В новом терминале или запустив по иконке:

```bash
cd planner/
./APM_Planner.AppImage 
