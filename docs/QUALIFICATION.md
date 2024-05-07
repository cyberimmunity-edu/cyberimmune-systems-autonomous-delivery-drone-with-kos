# Квалификация

Квалификация представляет собой выполнение нескольких заданий подтверждающих, что есть необходимые знания технологий, требуемых для соревнований и умений их использовать.

## Задания

Задания проверяют, что есть (или приобретены):

- базовые знания операционной системы на базе ядра [Linux](http://heap.altlinux.org/modules/linux_intro/index.html) и умение ей пользоваться
- базовые знания системы контроля версий [Git](https://git-scm.com/book/en/v2) и умение ей пользоваться
- базовые знания и умение пользоваться командной оболочкой и интерпретатором [Bash](https://www.gnu.org/software/bash/manual/bash.html)
- базовые знания C/C++

### Настройка и запуск цифрового двойника

Требуется:

1. Ознакомиться с [видео демонстрацией цифрового двойника и имитатора ОрВД](https://youtu.be/ytzJ13hsMwg?t=265).
2. настроить окружение на базе операционной системы Linux (рекомендуется Ubuntu 22.04)
3. сделать fork (в терминологии Git) main ветки проекта https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos
4. запустить цифрового двойника (в режиме offline и online)
5. пролететь виртуальную миссию отслеживая происходящее в планировщике

### Вывести отображение координат и высоты местоположения квадрокоптера в данный момент

Требуется:

1. ознакомиться с [программным интерфейсом](https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos/blob/main/README.kos.md) бортового модуля безопасности
2. используя вызовы программного интерфейса, отобразить в текстовом выводе координаты и высоту местоположения квадрокоптера в каждый момент (не чаще раза в секунду)
    - подсказка: редактировать понадобится только файл `kos/flight_controller/src/main.cpp`

## Рабочее окружение

Возможны варианты рабочего окружения:

1. Ubuntu-хост. На компьютере установлена ОС [Ubuntu 22.04](https://releases.ubuntu.com/jammy/) (или любой другой дистрибутив, но работоспособность в них не проверялась) и KasperskyOS Community Edition SDK установлен на хост систему.
2. Docker. На компьютере установлена ОС [Ubuntu 22.04](https://releases.ubuntu.com/jammy/) (или любой другой дистрибутив), [Docker](https://docs.docker.com/get-started/overview/), [Docker Compose](https://docs.docker.com/compose/) и KasperskyOS Community Edition SDK вместе с цифровым двойником запускается в контейнерах docker.
3. VirtualBox. На компьютере установлена любая ОС (Windows/Linux) и установлен [VirtualBox](https://www.virtualbox.org/) (образ настроенной ОС Ubuntu 22.04 предоставляется организаторами в чате) и работа с KasperskyOS Community Edition SDK будет происходить в виртуальной машине.
4. Windows. На компьютере установлена ОС из семейства Windows, [WSL 2](https://learn.microsoft.com/ru-ru/windows/wsl/install) и Ubuntu 22.04 внутри, [Docker Desktop](https://www.docker.com/products/docker-desktop/)

Описанные рабочие окружения тестировались и подойдут для выполнения задания и дальнейшем участие в соревнованиях.
