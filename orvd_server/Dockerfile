FROM ubuntu:22.04

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
        python3-flask \
        python3-jinja2 && \
        mkdir -p /var/www/orvd

COPY ./orvd_server.py /var/www/orvd/orvd_server.py
COPY ./orvd_server.wsgi /var/www/orvd/orvd_server.wsgi
COPY ./orvd.conf.docker /etc/apache2/sites-available/orvd.conf

RUN a2ensite orvd.conf && rm /etc/apache2/sites-available/000-default.conf

CMD apachectl -D FOREGROUND
