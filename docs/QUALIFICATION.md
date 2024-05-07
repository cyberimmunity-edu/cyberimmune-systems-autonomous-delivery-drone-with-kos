# Квалификация

Квалификация представляет собой выполнение нескольких заданий подтверждающих, что есть необходимые знания технологий требуемых для соревнований и умений их использовать.

## Задания

Задания провряют, что есть (или приобретены):

- базовые знания операционной системы на базе ядра [Linux](http://heap.altlinux.org/modules/linux_intro/index.html) и умения ей пользоваться
- базовые знания системы контроля версий [Git](https://git-scm.com/book/en/v2) и умения ей пользоваться
- базовые знания и умения пользоваться командной оболочки и интерпретатора [Bash](https://www.gnu.org/software/bash/manual/bash.html)
- базовые знания C/C++

### Настройка и запуск цифрового двойника

Требуется:

1. настроить окружение на базе операционной системы Linux (Ubuntu 22.04 рекомендуется)
1. сделать fork (в терминологии Git) main ветки проекта https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos
1. запустить цифрового двойника (в режиме offline и online)
1. пролететь виртуальную миссию отслеживая происходящее в планировщике

### Вывести отображение координат и высоты местоположения квадрокоптера в данный момент

Требуется:

1. ознакомиться с [программным интерфейсом](https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos/blob/main/README.kos.md) нашего модуля безопасности
1. используя вызовы программного интерфейса отобразить в текстовом выводе координаты и высоту местоположения квадракоптера в каждый момент (не чаще раза в секунду)
    - редактировать только файл `kos/flight_controller/src/main.cpp`

## Рабочее окружение

Возможны варианты рабочего окружения:

1. Ubuntu-хост. На компьютере установлена ОС [Ubuntu 22.04](https://releases.ubuntu.com/jammy/) (или любой другой дистрибутив, но на свой страх и риск) и KasperskyOS SDK установлен на хост систему.
2. Docker. На компьютере установлена ОС [Ubuntu 22.04](https://releases.ubuntu.com/jammy/) (или любой другой дистрибутив), [Docker](https://docs.docker.com/get-started/overview/), [Docker Compose](https://docs.docker.com/compose/) и KasperskyOS SDK вместе с цифровым двойником запускается в контейнерах docker.
3. VirtualBox. На компьютере установлена любая ОС (Windows/Linux) и установлен [VirtualBox](https://www.virtualbox.org/) (образ настроенной ОС Ubuntu 22.04 предоставляется организаторами в чате) и работа с KasperskyOS SDK будет происходит в виртуальной машине.
4. Windows. На компьютере установлена ОС из семейства Windows, [WSL 2](https://learn.microsoft.com/ru-ru/windows/wsl/install) и Ubuntu 22.04 внутри, [Docker Desktop](https://www.docker.com/products/docker-desktop/)

Описанные рабочие окружения тестировались и подойдут для выполнения задания и дальнейшем участие в соревнованиях.
