import sys

from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine, qmlRegisterSingletonInstance

from ConnectionsHandler import ConnectionsHandler
from utils import generate_keys, save_keys, KEY_SIZE


def main():
    public, private = generate_keys(KEY_SIZE)
    save_keys(public, private, 'mcc')
    app = QGuiApplication(sys.argv)
    engine = QQmlApplicationEngine()
    
    connectionsHandler = ConnectionsHandler()
    qmlRegisterSingletonInstance(ConnectionsHandler, 'G.ConnectionsHandler', 1, 0, 'ConnectionsHandler', connectionsHandler)
    connectionsHandler.start_server_socket_loop()
    
    engine.load('./QML/Main.qml')
    if not engine.rootObjects():
        sys.exit(-1)
    app.exec()
    
    connectionsHandler.close()


if __name__ == '__main__':
    main()


