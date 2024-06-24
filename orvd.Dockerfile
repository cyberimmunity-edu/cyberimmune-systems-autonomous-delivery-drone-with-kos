FROM cr.yandex/mirror/ubuntu:22.04

ENV DEBIAN_FRONTEND noninteractive
ENV FLASK_APP orvd_server.py
RUN apt-get update && \
    apt install -y \
        apache2 \
        libapache2-mod-wsgi-py3 \
        net-tools \
        python3 \
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
        python3-jinja2 && \
        mkdir -p /var/www/orvd

COPY ./orvd /var/www/orvd
COPY ./orvd.conf.docker /etc/apache2/sites-available/orvd.conf

RUN cd /var/www/orvd \
    && flask db init \
    && a2ensite orvd.conf \
    && rm /etc/apache2/sites-available/000-default.conf \
    && echo export ADMIN_LOGIN=admin >> /etc/apache2/envvars \
    && echo export ADMIN_PASSW=passw >> /etc/apache2/envvars \
    && sed -i -e 's/ErrorLog.*$/ErrorLog \/dev\/stderr/g' /etc/apache2/apache2.conf \
    && echo "Listen 8080" >> /etc/apache2/ports.conf \
    && chmod -R 777 /var/www/orvd

CMD apachectl -D FOREGROUND
