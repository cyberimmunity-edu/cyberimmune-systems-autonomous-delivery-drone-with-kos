# cyberimmune-systems-autonomous-delivery-drone-with-kos
Cyberimmune autonomous delivery drone prototype

## Развертывание HTTP-сервера ОРВД на виртуальной машине

Скачать:
- https://www.virtualbox.org/
- https://ubuntu.com/download/server

В VirtualBox создаем новую виртуальную машину, в качестве ISO Image указываем образ сервера Ubuntu.
Устанавливаем Ubuntu Server стандарным путем.
В настройках ВМ во вкладке Network выставляем Bridged Adapter вместо NAT. В свойствах сети Windows выставляем сетевой профиль "Частные" (если еще не выставлен).
Ждем окончания установки и перезагружаем.
Далее прописываем:
```
cd ~
git clone https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos.git
cd ./cyberimmune-systems-autonomous-delivery-drone-with-kos/orvd
sudo chmod +x install.sh update.sh restart.sh
sudo ./install.sh
```
Теперь сервер доступен по IP виртуальной машины. IP адрес машины можно получить через ifconfig.

Обновить сервер можно выполнив команду `sudo ./update.sh`, перезапустить - командой `sudo ./restart.sh`.