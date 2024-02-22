# cyberimmune-systems-autonomous-delivery-drone-with-kos
Cyberimmune autonomous delivery drone prototype

## Разворот клиента ОРВД
```
cd orvd_client
python -m venv .\.venv
.\.venv\Scripts\activate
pip install -r requirements.txt
```

Для python >=3.10 заменить collections.MutableMapping на collections.abc.MutableMapping в файле `.\.venv\Lib\site-packages\dronekit\__init__.py` строка 2689.

Запуск с помощью launch.bat

## Разворот HTTP-сервера ОРВД на виртуальной машине
```
cd orvd_server
```
Скачать:
- https://www.virtualbox.org/
- https://ubuntu.com/download/server
- PuTTY

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
sudo ufw allow 22/tcp
```

Выключаем ВМ. В настройках ВМ во вкладке Network выставляем Bridged Adapter вместо NAT. В свойствах сети Windows выставляем сетевой профиль "Частные" (если еще не выставлен).
Включаем ВМ. Через ifconfig получаем IP адрес машины.
Подключаемся к ВМ через PuTTY по SSH по этому IP.
Далее прописываем:
```
sudo apt-get install python3 python3-pip python3-venv
sudo apt-get install libapache2-mod-wsgi-py3

cd /var/www && sudo mkdir orvd
cd orvd
sudo python3 -m venv .venv
source .venv/bin/activate
sudo chmod -R 777 /var/www
pip3 install flask

sudo nano orvd_server.py
```
Копируем сюда содержимое orvd_server.py и сохраняем. Значение HOST заменяем на IP-адрес клиента ОРВД.

```
export FLASK_APP=orvd_server.py
deactivate

sudo nano /var/www/orvd/orvd_server.wsgi
```
Копируем сюда содержимое orvd_server.wsgi и сохраняем.
```
sudo touch /etc/apache2/sites-available/orvd.conf
sudo nano /etc/apache2/sites-available/orvd.conf
```
Копируем сюда содержимое orvd.conf и сохраняем.
```
sudo a2ensite orvd.conf
sudo rm sudo rm /etc/apache2/sites-available/000-default.conf
sudo systemctl restart apache2
```
Теперь сервер доступен по IP виртуальной машины.
