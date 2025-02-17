#!/bin/bash
git checkout orvd --
git pull
rm -r /var/www/orvd
cp -r ./ /var/www/orvd
chmod -R 777 /var/www/orvd
systemctl restart apache2