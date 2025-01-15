# Программный интерфейс компонента безопасности

Оглавление:

- [Структура контроллера KOS](#структура-контроллера-kos)
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
  - [ipc_messages_logger](#ipc_messages_logger)
    - [logEntry()](#int-logentrychar-entry-char-entity-loglevel-level)
  - [ipc_messages_navigation_system()](#ipc_messages_navigation_system)
    - [getCoords()](#int-getcoordsint32_t-latitude-int32_t-longitude-int32_t-altitude)
    - [getGpsInfo()](#int-getgpsinfofloat-dop-int32_t-sats)
    - [getEstimatedSpeed()](#int-getestimatedspeedfloat-speed)
  - [ipc_messages_periphery_controller](#ipc_messages_periphery_controller)
    - [enableBuzzer()](#int-enablebuzzer)
    - [setKillSwitch()](#int-setkillswitchuint8_t-enable)
    - [setCargoLock()](#int-setcargolockuint8_t-enable)
  - [ipc_messages_server_connector](#ipc_messages_server_connector)
    - [getBoardId()](#int-getboardidchar-id)
    - [sendRequest()](#int-sendrequestchar-query-char-response)
    - [publishMessage()](#int-publishmessagechar-topicchar-publication)

## Структура контроллера KOS

Компонент безопасности или контроллер состоит из 7 модулей:

- autopilot_connector
- credential_manager
- navigation_system
- periphery_controller
- server_connector
- logger
- **flight_controller**

Участникам соревнований предлагается вносить изменения в программный код модуля **flight_controller** (Находится в папке kos/flight_controller). Взаимодействие с остальными модулями выполняется посредством IPC-сообщений; внесение изменений в эти модули не рекомендуется.

## IPC-сообщения

Папка shared содержит файлы с функциями-обертками, облегчающими отправку IPC-сообщений. Файлы соответствуют модулям, для которгго предназачены сообщения. Все функции-обертки возвращают цифровое значение: **1** в случае успешной отправки и получении ответа, **0**  в ином случае.

### `ipc_messages_initialization`

Содержит универсальное сообщение, воспринимаемое модулями AutopilotConnector, CredentialManager, Logger,
NavigationSystem, PeripheryController и ServerConnector.

#### `int waitForInit(const char* connection, const char* receiverEntity)`

Ожидает инициализации указанного модуля. Требует указания имени модуля и имени соединения, по которому будет отправлено сообщение.

### `ipc_messages_autopilot_connector`

Cодержит сообщения для модуля AutopilotConnector, выполняющего взаимодействие с автопилотом через UART-порт.

#### `int waitForArmRequest()`

Переходит в режим ожидания arm-запроса от автопилота. Не завершится, пока не получит сообщение от автопилота.

#### `int permitArm()`

Передает в автопилот сообщение разрешении на arm. Данная команда не заставляет автопилот произвести arm.

#### `int forbidArm()`

Передает в автопилот сообщение запрет на arm. Данная команда не предотвращает дальнейшие попытки автопилота произвести arm.

#### `int pauseFlight()`

Передает в автопилот запрос на приостановку задания. При корректной работе автопилота это вызовет прерывание текущей миссии и остановку текущем местоположении дрона.

#### `int resumeFlight()`

Передает в автоплиот запрос на возобновление задания. При корректной работе автопилота это приведет к возобновлению приостановленной ранее миссии.

#### `int changeSpeed(int32_t speed)`

Передает в автоплиот запрос на изменение скорости. Скорость указывается в м/с.

#### `int changeAltitude(int32_t altitude)`

Передает в автоплиот запрос на изменение высоты для всех точек миссии. Высота указывается в см относительно высоты точки дома. Прошивка автономной машинки проигнорирует данную команду.

#### `int changeWaypoint(int32_t latitude, int32_t longitude, int32_t altitude)`

Передает в автопилот запрос на изменение текущей точки миссии. Высота указывается в см относительно высоты дома; широта и долгота - в градусах \* 10^7. Несмотря на необходимость указывать все три координаты, прошивка машники будет учитывать лишь долготу и широту переданной точки.

### `ipc_messages_credential_manager`

Содержит сообщения для модуля CredentialManager, отвечающего за RSA-шифрование сообщений для сервера.

#### `int signMessage(char* message, char* signature)`

Производит вычисление RSA-подписи сообщения message, возвращая ее в signature.

#### `int checkSignature(char* message, uint8_t &authenticity)`

Производит проверку аутентичности сообщения, поданного в формате "сообщение#подпись". Результат проверки аутентичности возвращается в authenticity.

### `ipc_messages_logger`

Содержит сообщения для модуля Logger, отвечающего за сохранение логов на карту памяти и их вывод в консоль.

#### `int logEntry(char* entry, char* entity, LogLevel level)`

Выполняет запись сообщения в лог с указанием модуля, отправившего сообщение, и уровня логирования.

### `ipc_messages_navigation_system`

Содержит сообщения для модуля NavigationSystem, работающего с GPS и барометром модуля безопасности.

#### `int getCoords(int32_t &latitude, int32_t &longitude, int32_t &altitude)`

Возвращает текущие координаты дрона. Значения долготы и широты - в градусах \* 10^7, высоты - в см.

#### `int getGpsInfo(float& dop, int32_t &sats)`

Возвращает значение DOP (снижение точности) и число наблюдаемых спутников (sats).

#### `int getEstimatedSpeed(float& speed)`

Возвращает скорость (в м/с), определенную модулем GNSS.

### `ipc_messages_periphery_controller`

Содержит сообщения для модуля PeripheryController, выполняющего взаимодействие с периферией через GPIO.

#### `int enableBuzzer()`

Включает зуммер. Звук будет автоматически отключен через 2 секунды.

#### `int setKillSwitch(uint8_t enable)`

Регулирует подачу питания на двигатели. При 1 использование моторов возможно, при 0 - нет.

#### `int setCargoLock(uint8_t enable)`

Регулирует подачу питания на мотор сброса груза. При 1 сброс груза возможен, при 0 - нет.

### `ipc_messages_server_connector`

Содержит сообщения для модуля ServerConnector, выполняющего общение я сервером СУПА.

#### `int getBoardId(char* id)`

Возвращает ID дрона, соответствующий MAC-адресу интерфейса "en0".

#### `int sendRequest(char* query, char* response)`

Отправляет на сервер запрос, возвращая в response значимое содержание полученного ответа.

#### `int publishMessage(char* topic, char* publication)`

Публикует сообщение с указанной темой посредством протокола MQTT; ответ от сервера СУПА не ожидается.