# ARDUPILOT

Модифицированная версия проишвки ArduPilot находится в ветке "ardupilot" этого репозитория. Модифицированная версия содержит необходиые дополнения, обеспечивающие взаимодействие автопилота с модулем безопасности. Также модифицированная версия содержит изменения, необходимые для взаимодействия цифрового двойника с модулем безопасности.

Версия ArduPilot, содержащая реализацию киберпрепятствий, является закрытой.

[Документация по проекту ArduPilot](https://ardupilot.org/dev/index.html)

## Установка зависимостей

```bash
cd cyberimmune-systems-autonomous-delivery-drone-with-kos
git switch ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

## Сборка прошивки

Сборка прошивки под полетный контроллер SkyStars выполняется следующим способом:

```bash
cd cyberimmune-systems-autonomous-delivery-drone-with-kos
git switch ardupilot
./waf configure --board SkystarsH7HD-bdshot
./waf copter
```

После выполнения данной команды в папке "cyberimmune-systems-autonomous-delivery-drone-with-kos/build/SkystarsH7HD-bdshot/bin" будет находиться собранная прошивка ArduPilot. Ее можно использовать для записи в полетный контроллер дрона.

## Сборка цифрового двойника

Цифровой двойник также собирается на основе ArduPilot. Сборка SITL-прошивки выполняется следующим образом:

```bash
cd cyberimmune-systems-autonomous-delivery-drone-with-kos
git switch ardupilot
cd ArduCopter
sim_vehicle.py
```

Собранный цифровой двойник будет находиться в папке "cyberimmune-systems-autonomous-delivery-drone-with-kos/build/sitl/bin". Для использования собранного цифрового двойника, необходимо заменить им файл "cyberimmune-systems-autonomous-delivery-drone-with-kos/ardupilot/sitl/arducopter" в ветке main.