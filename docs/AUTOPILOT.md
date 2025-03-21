# ARDUPILOT

Модифицированная версия Ardupilot находится в ветке "ardupilot" этого репозитория. Модифицированная версия содержит необходиые дополнения, обеспечивающие взаимодействие автопилота с модулем безопасности. Также модифицированная версия содержит изменения, необходимые для взаимодействия цифрового двойника с модулем безопасности.

Ardupilot, содержащий реализацию киберпрепятствий, закрыт для участников соревнований.

[Документация по проекту ArduPilot](https://ardupilot.org/dev/index.html)

## Установка зависисмостей

Перед первой сборкой прошивки автопилота необходимо подготовить репозиторий с ним к работе:

```bash
cd cyberimmune-systems-autonomous-delivery-drone-with-kos
git switch ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

## Сборка прошивки

Сборка прошивки выполняется следующим способом:

```bash
cd cyberimmune-systems-autonomous-delivery-drone-with-kos
git switch ardupilot
./waf configure --board SkystarsH7HD-bdshot
./waf copter
```

После выполнения данной команды в папке "cyberimmune-systems-autonomous-delivery-drone-with-kos/build/SkystarsH7HD-bdshot/bin" будет находиться собранный Ardupilot. Его можно использовать для прошивки полетного контроллера дрона.

## Сборка цифрового двойника

Цифровой двойник также собирается на основе Ardupilot. Сборка SITL-прошивки выполняется следующим образом:

```bash
cd cyberimmune-systems-autonomous-delivery-drone-with-kos
git switch ardupilot
cd ArduCopter
sim_vehicle.py
```

Собранный цифровой двойник будет находиться в папке "cyberimmune-systems-autonomous-delivery-drone-with-kos/build/sitl/bin". Для использования собранного цифрового двойника, необходимо заменить им файл "cyberimmune-systems-autonomous-delivery-drone-with-kos/ardupilot/sitl/arducopter" в ветке main.