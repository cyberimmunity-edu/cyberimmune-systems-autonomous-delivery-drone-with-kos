#!/bin/bash
apt-get update
apt install -y apache2 net-tools
ufw enable
ufw allow OpenSSH
ufw allow Apache
ufw allow 8080/tcp
apt-get -y install python3 \
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

apt-get -y install libapache2-mod-wsgi-py3
cp -r ./ /var/www/orvd
cd /var/www/orvd
export FLASK_APP=orvd_server.py
export ADMIN_LOGIN=admin
export ADMIN_PASSW=passw
chmod -R 777 /var/www
flask db init
cp ./orvd.conf /etc/apache2/sites-available/orvd.conf
cp ./ports.conf /etc/apache2/ports.conf
a2ensite orvd.conf
rm /etc/apache2/sites-available/000-default.conf

sed -i '$a export ADMIN_LOGIN='$ADMIN_LOGIN /etc/apache2/envvars
sed -i '$a export ADMIN_PASSW='$ADMIN_PASSW /etc/apache2/envvars

systemctl restart apache2

