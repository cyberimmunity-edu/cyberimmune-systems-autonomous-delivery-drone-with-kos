# Программный интерфейс компонента безопасности

Оглавление:

- [Структура полетного контроллера KOS](#структура-полетного-контроллера-kos)
- [IPC-сообщения](#ipc-сообщения)
  - [ipc_messages_initialization](#ipc_messages_initialization)
    - [waitForInit()](#int-waitforinitconst-char-connection-const-char-receiverentity)
  - [ipc_messages_autopilot_connector](#ipc_messages_autopilot_connector)
    - [waitForArmRequest()](#int-waitforarmrequest)
    - [permitArm()](#int-permitarm)
    - [forbidArm()](#int-forbidarm)
    - [pauseFlight()](#int-pauseflight)
    - [resumeFlight()](#int-resumeflight)
    - [changeSpeed()](#int-changespeedint32_t-speed)
    - [changeAltitude()](#int-changealtitudeint32_t-altitude)
    - [changeWaypoint()](#int-changewaypointint32_t-latitude-int32_t-longitude-int32_t-altitude)
  - [ipc_messages_credential_manager](#ipc_messages_credential_manager)
    - [signMessage()](#int-signmessagechar-message-char-signature)
    - [checkSignature()](#int-checksignaturechar-message-uint8_t-authenticity)
  - [ipc_messages_navigation_system()](#ipc_messages_navigation_system)
    - [getCoords()](#int-getcoordsint32_t-latitude-int32_t-longitude-int32_t-altitude)
    - [getGpsInfo()](#int-getgpsinfofloat-dop-int32_t-sats)
  - [ipc_messages_periphery_controller](#ipc_messages_periphery_controller)
    - [enableBuzzer()](#int-enablebuzzer)
    - [setKillSwitch()](#int-setkillswitchuint8_t-enable)
    - [setCargoLock()](#int-setcargolockuint8_t-enable)
  - [ipc_messages_server_connector](#ipc_messages_server_connector)
    - [sendRequest()](#int-sendrequestchar-query-char-response)

## Структура полетного контроллера KOS

Компонент безопасности или полётный контроллер состоит из 6 модулей:

- autopilot_connector
- credential_manager
- navigation_system
- periphery_controller
- server_connector
- **flight_controller**

Участникам соревнований предлагается вносить изменения в программный код модуля **flight_controller** (Находится в папке kos/flight_controller). Взаимодействие с остальными модулями выполняется посредством IPC-сообщений; внесение изменений в эти модули не рекомендуется.

## IPC-сообщения

Папка shared содержит файлы с функциями-обертками, облегчающими отправку IPC-сообщений. Файлы соответствуют модулям, для которгго предназачены сообщения. Все функции-обертки возвращают цифровое значение: **1** в случае успешной отправки и получении ответа, **0**  в ином случае.

### `ipc_messages_initialization`

Содержит универсальное сообщение, воспринимаемое модулями autopilot_connector, credential_manager, navigation_system, periphery_controller и server_connector.

#### `int waitForInit(const char* connection, const char* receiverEntity)`

Статус инициализации модуля. Поскольку сообщение универсально, необходимо указать модуль и соединение, по которому будет отправлено сообщение.

### `ipc_messages_autopilot_connector`

Cодержит сообщения для модуля autopilot_connector, выполняющего взаимодействие с прошивкой автопилота через UART-порт.

#### `int waitForArmRequest()`

Переводит UART-порт в режим "слушания" и ожидает получение от автопилота запроса на arm.

#### `int permitArm()`

Передает в автопилот сообщение, разрешающее arm. Данная команда не заставляет автопилот произвести arm.

#### `int forbidArm()`

Передает в автопилот сообщение, запрещающее arm. Данная команда не гарантирует, что arm в автопилоте не будет произведен.

#### `int pauseFlight()`

Передает в автопилот запрос на приостановку полета. При корректной работе автопилота это вызовет прерывание текущей миссии и выполнение посадки в текущем местоположении дрона.

#### `int resumeFlight()`

Передает в автоплиот запрос на возобновление полета. При корректной работе автопилота это приведет к возобновлению приостановленной ранее миссии.

#### `int changeSpeed(int32_t speed)`

Передает в автоплиот запрос на изменение скорости. Скорость указывается в м/с

#### `int changeAltitude(int32_t altitude)`

Передает в автоплиот запрос на изменение высоты для всех точек миссии. Высота указывается в см относительно высоты дома

#### `int changeWaypoint(int32_t latitude, int32_t longitude, int32_t altitude)`

Передает в автопилот запрос на изменение текущей точки мисси на переданную. Высота указывается в см относительно высоты дома; щирота и долгота - в градусах \* 10^7

### `ipc_messages_credential_manager`

Содержит сообщения для модуля credential_manager, выполняющего проверку аутентичности подписей сообщений от ОРВД

#### `int signMessage(char* message, char* signature)`

Производит вычисление подписи для сообщения message и записывает ее в signature

#### `int checkSignature(char* message, uint8_t &authenticity)`

Производит проверку аутентичности сообщения. На выход подается сообщение в формате "сообщение#подпись" (без кавычек), записывает в authenticity **1** в случае подтверждения аутентичности и **0** в ином случае

### `ipc_messages_navigation_system`

Содержит сообщения для модуля navigation_system, работающего с GPS и баромтером, подключенным к RaspberryPi

#### `int getCoords(int32_t &latitude, int32_t &longitude, int32_t &altitude)`

Записывает в переданные переменные текущие координаты дрона. Значения долготы и широты - в градусах \* 10^7, высоты - в см

#### `int getGpsInfo(float& dop, int32_t &sats)`

Записывает в переданные переменные значение DOP (снижение точности) и число наблюдаемых спутников (sats), полученные от GPS

### `ipc_messages_periphery_controller`

Содержит сообщения для модуля periphery_controller, выполняющего взаимодействие с GPIO

#### `int enableBuzzer()`

Включает баззер. Звук будет автоматически отключен через 2 секунды

#### `int setKillSwitch(uint8_t enable)`

Регулирует подачу питания на двигатели. При enbale=1 использование моторов возможно, при 0 - нет.

#### `int setCargoLock(uint8_t enable)`

Регулирует подачу питания на двигатель сброса груза. При enbale=1 сброс груза возможен, при 0 - нет.

### `ipc_messages_server_connector`

Содержит сообщения для модуля server_connector, выполняющего общение я сервером ОРВД.

#### `int sendRequest(char* query, char* response)`

Отправляет на сервер запрос, переданный в request, и записывает ответ в response. В response возвращается только значимое содержание полученного ответа.
