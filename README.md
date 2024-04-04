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
sudo apt install apache2
sudo apt install net-tools
sudo ufw enable
sudo ufw allow OpenSSH
sudo ufw allow Apache
```
Выключаем ВМ. В настройках ВМ во вкладке Network выставляем Bridged Adapter вместо NAT. В свойствах сети Windows выставляем сетевой профиль "Частные" (если еще не выставлен).

Включаем ВМ. Через ifconfig получаем IP адрес машины.
Подключаемся по SSH по этому IP (например, через VS Code).

Далее устанавливаем MySQL. Вместо exampleuser и examplepassword указываем желаемый логин и пароль от базы:
```
sudo apt install mysql-server
sudo mysql_secure_installation

sudo mysql
CREATE USER 'exampleuser'@'localhost' IDENTIFIED BY 'examplepassword';
CREATE DATABASE orvd;
GRANT ALL PRIVILEGES ON *.* TO 'exampleuser'@'localhost' WITH GRANT OPTION;
exit
```
Устанавливаем Python и формируем директорию.
```
sudo apt-get install python3 python3-pip python3-venv
sudo apt-get install libapache2-mod-wsgi-py3
cd /var/www && sudo mkdir orvd
cd orvd
sudo chmod -R 777 /var/www
```
Копируем файлы из orvd в `/var/www/orvd`. Далее прописываем:
```
export FLASK_APP=orvd_server.py
export DATABASE_URL=exampleuser:examplepassword@localhost:3306
export ADMIN_LOGIN=admin
export ADMIN_PASSW=passw
```
Вместо exampleuser и examplepassword указываем логин и пароль от базы, вместо admin и passw - желаемые логин и пароль от ОРВД.
```
sudo python3 -m venv .venv
source .venv/bin/activate
sudo chmod -R 777 /var/www
pip3 install -r requirements.txt
flask db init
deactivate
```
Далее прописываем:
```
sudo cp ./orvd.conf /etc/apache2/sites-available/orvd.conf
sudo a2ensite orvd.conf
sudo rm /etc/apache2/sites-available/000-default.conf

sudo sed -i '$a export DATABASE_URL='$DATABASE_URL /etc/apache2/envvars
sudo sed -i '$a export ADMIN_LOGIN='$ADMIN_LOGIN /etc/apache2/envvars
sudo sed -i '$a export ADMIN_PASSW='$ADMIN_PASSW /etc/apache2/envvars

sudo systemctl restart apache2
```
Теперь сервер доступен по IP виртуальной машины.