# cyberimmune-systems-autonomous-delivery-drone-with-kos
Cyberimmune autonomous delivery drone prototype

## Развертывание HTTP-сервера ОРВД на виртуальной машине
```
cd orvd
```
Скачать:
- https://www.virtualbox.org/
- https://ubuntu.com/download/server

В VirtualBox создаем новую виртуальную машину, в качестве ISO Image указываем образ сервера Ubuntu.
Устанавливаем Ubuntu Server стандарным путем. Ждем окончания установки и перезагружаем.
Далее прописываем:
```
sudo apt update
sudo apt install apache2 net-tools
sudo ufw enable
sudo ufw allow OpenSSH
sudo ufw allow Apache
```
Выключаем ВМ. В настройках ВМ во вкладке Network выставляем Bridged Adapter вместо NAT. В свойствах сети Windows выставляем сетевой профиль "Частные" (если еще не выставлен).

Включаем ВМ. Через ifconfig получаем IP адрес машины.
Подключаемся по SSH по этому IP (например, через VS Code).

Устанавливаем Python и формируем директорию.
```
sudo apt-get install python3 \
        python3-pip \
        python3-venv \
        python3-blinker \
        python3-flask \
        python3-flask-migrate \
        python3-flask-sqlalchemy \
        python3-greenlet \
        python3-itsdangerous \
        python3-mako \
        python3-markupsafe \
        python3-pycryptodome \
        python3-typing-extensions \
        python3-werkzeug \
        python3-jinja2

sudo apt-get install libapache2-mod-wsgi-py3
cd /var/www && sudo mkdir orvd && cd orvd
sudo chmod -R 777 /var/www
```
Копируем файлы из orvd в `/var/www/orvd`. Далее прописываем:
```
export FLASK_APP=orvd_server.py
export ADMIN_LOGIN=admin
export ADMIN_PASSW=passw
```
Вместо admin и passw указываем желаемые логин и пароль от ОРВД.
```
flask db init
sudo chmod -R 777 /var/www/orvd
```
Далее прописываем:
```
sudo cp ./orvd.conf /etc/apache2/sites-available/orvd.conf
sudo a2ensite orvd.conf
sudo rm /etc/apache2/sites-available/000-default.conf

sudo sed -i '$a export ADMIN_LOGIN='$ADMIN_LOGIN /etc/apache2/envvars
sudo sed -i '$a export ADMIN_PASSW='$ADMIN_PASSW /etc/apache2/envvars

sudo systemctl restart apache2
```
Теперь сервер доступен по IP виртуальной машины.