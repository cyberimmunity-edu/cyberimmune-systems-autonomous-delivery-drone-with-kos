# Установки пакетов и запуска цифрового двойника для VirtualBox Ubuntu 22.04 

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

### Запуск APM Planner 2

```bash
cd planner/
./APM_Planner.AppImage 
```
# Результаты
### Для решения поставленной задачи, был модернизирован файл main.cpp.
#### Согласно документации, добавили в конец файла код вывода долготы, широты и высоты, значения которых были описаны в файле докумменташки API.

```bash
# Код 
  while (true) {
    int32_t lantitude, longitude, altitude;
    getCoords(lantitude, longitude, altitude);
    fprintf(stderr, "Info: [%d].[%d] : [%d]\n", lantitude, longitude, altitude);
    sleep(2);
  } 
```

#### Прогнали через тесты, все работает, ОК :)

![image](https://github.com/St1nk0/cyberimmune-systems-autonomous-delivery-drone-with-kos/assets/130299705/0fb44e9d-d27d-4346-81a8-7518bbd965e0)

#### Так же попробывали вывести различные переменные и сообщения в других циклах\условиях while. Потестировали как это работает не только в оффлайн режиме, но и онлайн.

[![image](https://github.com/St1nk0/cyberimmune-systems-autonomous-delivery-drone-with-kos/assets/130299705/4010f99a-8e7f-40e7-ae74-66e9e74e74d7)
![image](https://github.com/St1nk0/cyberimmune-systems-autonomous-delivery-drone-with-kos/assets/130299705/5def7ded-5c9e-488c-87b2-dd5e2f3f588b)]

![image](https://github.com/St1nk0/cyberimmune-systems-autonomous-delivery-drone-with-kos/assets/130299705/78a2a7a7-a559-4fb9-8829-668e20afa688)

![image](https://github.com/St1nk0/cyberimmune-systems-autonomous-delivery-drone-with-kos/assets/130299705/c18c894d-8eb9-435d-b0e5-c11e96c72bd6)

![image](https://github.com/St1nk0/cyberimmune-systems-autonomous-delivery-drone-with-kos/assets/130299705/fdf94290-bb8b-4415-9879-9d0abfa95fd4)





