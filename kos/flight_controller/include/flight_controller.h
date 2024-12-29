/**
 * \file
 * \~English \brief Declaration of methods for flight mission management.
 * \~Russian \brief Объявление методов для работы с полетной миссией.
 */

#pragma once
#include <stdint.h>

/** \cond */
#define AREA_NAME_MAX_LEN 32
/** \endcond */

/**
 * \~English Mission command type recognized by the security module.
 * \~Russian Тип распознаваемой модулем безопасности команды миссии.
 */
enum CommandType {
    /**
     * \~English Set provided coordinates as home. Thhis command is expected to be the first in the mission.
     * \note The actual home point may differ from set one.
     * \~Russian Назначение заданной точки домом. Ожидается, что эта команда -- первая в миссии.
     * \note Фактическое место дома может не совпадать с указанным в миссии.
     */
    HOME,
    /**
     * \~English Command to start flight by taking specified altitude. Expected to follow \ref HOME command.
     * \~Russian Предписание подняться на заданную высоту. Ожидается, что эта команда следует за командой \ref HOME.
     */
    TAKEOFF,
    /**
     * \~English Command to fly to specified point.
     * \~Russian Предписание лететь к указанной точке.
     */
    WAYPOINT,
    /**
     * \~English Command to land at specified point.
     * \note If specified latitude and longitude are equal zero, the landing will be performed at current drone location.
     * \~Russian Предписание приземлиться в указанной точке.
     * \note Если указанные широта и долгота равны нулю, посадка будет произведена в текущем местонахождении дрона.
     */
    LAND,
    /**
     * \~English Set specified PWM value for specified servo.
     * \~Russian Установка указанного значения ШИМ для указанного сервопривода.
     */
    SET_SERVO,
    /**
     * \~English Command to wait for specified amount of seconds before next command execution.
     * \~Russian Ожидание указанног очисла секунд перед выполнением следующей команды.
     */
    DELAY
};

/**
 * \~English Structure to store \ref TAKEOFF command arguments.
 * \~Russian Структура для хранения аргументов команды \ref TAKEOFF.
 */
struct CommandTakeoff {
    /**
     * \~English Altitude relative to home point in cm.
     * \~Russian Высота относительно дома в см.
     */
    int32_t altitude;

    /**
     * \~English Default constructor.
     * \param[in] alt Relative altitude in cm.
     * \~Russian Конструктор по умолчанию.
     * \param[in] alt Относительная высота в см.
     */
    CommandTakeoff(int32_t alt) {
        altitude = alt;
    }
};

/**
 * \~English Structure to store \ref WAYPOINT command arguments (as well as \ref HOME and \ref LAND).
 * \~Russian Структура для хранения аргументов команды \ref WAYPOINT (а также \ref HOME и \ref LAND).
 */
struct CommandWaypoint {
    /**
     * \~English Latitude in degrees * 10^7.
     * \~Russian Широта в градусах * 10^7.
     */
    int32_t latitude;
    /**
     * \~English Longitude in degrees * 10^7.
     * \~Russian Долгота в градусах * 10^7.
     */
    int32_t longitude;
    /**
     * \~English Altitude in cm.
     * \note Relative for \ref WAYPOINT command, absolute -- for \ref HOME and \ref LAND.
     * \~Russian Высота в см.
     * \note Относительная для команды \ref WAYPOINT, абсолютная -- для \ref HOME и \ref LAND.
     */
    int32_t altitude;

    /**
     * \~English Default constructor.
     * \param[in] lat Latitude in degrees * 10^7.
     * \param[in] lng Longitude in degrees * 10^7.
     * \param[in] alt Altitude in cm.
     * \~Russian Конструктор по умолчанию.
     * \param[in] lat Широта в градусах * 10^7.
     * \param[in] lng Долгота в градусах * 10^7.
     * \param[in] alt Высота в см.
     */
    CommandWaypoint(int32_t lat, int32_t lng, int32_t alt) {
        latitude = lat;
        longitude = lng;
        altitude = alt;
    }
};

/**
 * \~English Structure to store \ref SET_SERVO command arguments.
 * \~Russian Структура для хранения аргументов команды \ref SET_SERVO.
 */
struct CommandServo {
    /**
     * \~English Servo number.
     * \note Number starts with 1. The first servos refer to drone motors.
     * \~Russian Номер сервопривода.
     * \note Нумерация начинается с 1. Первые сервоприводы отностяся к двигателям дрона.
     */
    int32_t number;
    /**
     * \~English PWM value.
     * \~Russian Значение ШИМ.
     */
    int32_t pwm;

    /**
     * \~English Default constructor.
     * \param[in] num Servo number.
     * \param[in] pwn PWM value.
     * \~Russian Конструктор по умолчанию.
     * \param[in] num Номер сервопривода.
     * \param[in] pwn Значение ШИМ.
     */
    CommandServo(int32_t num, int32_t pwm_) {
        number = num;
        pwm = pwm_;
    }
};

/**
 * \~English Structure to store \ref DELAY command arguments.
 * \~Russian Структура для хранения аргументов команды \ref DELAY.
 */
struct CommandDelay {
    /**
     * \~English Delay in seconds before next command.
     * \~Russian Задержка в секундах перед выполнением следующей команды.
     */
    int32_t delay;

    /**
     * \~English Default constructor.
     * \param[in] delay_ Delay in seconds.
     * \~Russian Конструктор по умолчанию.
     * \param[in] delay_ Задержка в секундах.
     */
    CommandDelay(int32_t delay_) {
        delay = delay_;
    }
};

/**
 * \~English Union to store any command arguments.
 * \~Russian Объединение для хранения аргументов команды любого типа.
 */
union CommandContent {
    /**
     * \~English \ref TAKEOFF command arguments. See \ref CommandTakeoff.
     * \~Russian Аргументы команды \ref TAKEOFF. См. \ref CommandTakeoff.
     */
    CommandTakeoff takeoff;
    /**
     * \~English \ref HOME, \ref WAYPOINT and \ref LAND command arguments. See \ref CommandWaypoint.
     * \~Russian Аргументы команд \ref HOME, \ref WAYPOINT и \ref LAND. См. \ref CommandWaypoint.
     */
    CommandWaypoint waypoint;
    /**
     * \~English \ref SET_SERVO command arguments. See \ref CommandServo.
     * \~Russian Аргументы команды \ref SET_SERVO. См. \ref CommandServo.
     */
    CommandServo servo;
    /**
     * \~English \ref DELAY command arguments. See \ref CommandDelay.
     * \~Russian Аргументы команды \ref DELAY. См. \ref CommandDelay.
     */
    CommandDelay delay;
};

/**
 * \~English Structure to store mission command with its arguments.
 * \~Russian Структура для хранения команды миссии с аргументами.
 */
struct MissionCommand {
    /**
     * \~English Command type. See \ref CommandType.
     * \~Russian Тип команды. См. \ref CommandType.
     */
    CommandType type;
    /**
     * \~English Command arguments. See \ref CommandContent.
     * \~Russian Аргументы команды. См. \ref CommandContent.
     */
    CommandContent content;
};

/**
 * \~English Structure to store coordinates of no-flight area vertex.
 * \~Russian Структура для хранения координат вершины бесполетной зоны.
 */
struct Point2D {
    /**
     * \~English Latitude in degrees * 10^7.
     * \~Russian Широта в градусах * 10^7.
     */
    int32_t latitude;
    /**
     * \~English Command type. See \ref CommandType.
     * \~Russian Longitude in degrees * 10^7.
     */
    int32_t longitude;

    /**
     * \~English Default constructor.
     * \param[in] lat Latitude in degrees * 10^7.
     * \param[in] lng Longitude in degrees * 10^7.
     * \~Russian Конструктор по умолчанию.
     * \param[in] lat Широта в градусах * 10^7.
     * \param[in] lng Долгота в градусах * 10^7.
     */
    Point2D(int32_t lat, int32_t lng) {
        latitude = lat;
        longitude = lng;
    }
};

/**
 * \~English Structure to store no-flight area.
 * \~Russian Структура для хранения бесполетной зоны.
 */
struct NoFlightArea {
    /**
     * \~English No-flight area name.
     * \~Russian Имя бесполетной зоны.
     */
    char name[AREA_NAME_MAX_LEN + 1];
    /**
     * \~English Number of vertices of the polygon that forms the no-flight area.
     * \~Russian Число вершин многоугольника, образующего бесполетную зону.
     */
    int pointNum;
    /**
     * \~English No-flight area vertices.
     * \~Russian Веришны бесполетной зоны.
     */
    Point2D* points;
};

/**
 * \~English Converts a mission received from the ATM server into an array of commands of \ref MissionCommand type.
 * \param[in] mission Mission from the ATM server. Expected form is "$FlightMission mission#".
 * \return Returns 1 on successful conversion, 0 otherwise.
 * \~Russian Преобразует миссию, полученную от сервера ОРВД, в массив команд типа \ref MissionCommand.
 * \param[in] mission Миссия, пришедшая от сервера ОРВД. Ожидается в виде "$FlightMission миссия#".
 * \return Возвращает 1, если миссия была успешно распознана, иначе -- 0.
 */
int loadMission(char* mission);
/**
 * \~English Logs an existing mission and at the same time prints it to the console.
 * \~Russian Записывает имеющуюся миссию в лог и одновременно выводит ее в консоль.
 */
void printMission();
/**
 * \~English Returns current drone mission.
 * \param[out] num Number of mission commands.
 * \return Returns pointer to a mission commands array.
 * \~Russian Возвращает текущую миссию дрона.
 * \param[out] num Число команд в миссии.
 * \return Возвращает указатель на массив команд миссии.
 */
MissionCommand* getMissionCommands(int &num);

/**
 * \~English Converts an array of commands to a string, that can be recognised by the ATM server.
 * \param[in] commands Array of commands to convert.
 * \param[in] num Number of commands to convert.
 * \param[in, out] string String, where the converted commands will be stored.
 * \param[in] len Length of string to write converted commands.
 * \return Returns 1 on successful conversion, 0 otherwise.
 * \~Russian Преобразует массив команд в строку, распознаваемую сервером ОРВД.
 * \param[in] commands Массив команд, которые будут конвертированы.
 * \param[in] num Количество конвертируемых команд.
 * \param[in, out] string Строка, куда будут записаны конвертированные команды.
 * \param[in] len Длина строки для записи результата.
 * \return Возвращает 1 при успешном преобразовании, иначе -- 0.
 */
int missionToString(MissionCommand* commands, uint8_t num, char* string, uint32_t len);
/**
 * \~English Converts an array of commands to a byte array, that can be recognised by the autopilot firmware.
 * \param[in] commands Array of commands to convert.
 * \param[in] num Number of commands to convert.
 * \param[out] bytes Byte array, where the converted commands will be stored.
 * \return Returns 1 on successful conversion, 0 otherwise.
 * \~Russian Преобразует массив команд в массив байтов, распознаваемый прошивкой автопилота.
 * \param[in] commands Массив команд, которые будут конвертированы.
 * \param[in] num Количество конвертируемых команд.
 * \param[out] bytes Массив байтов, куда будут записаны конвертированные команды.
 * \return Возвращает 1 при успешном преобразовании, иначе -- 0.
 */
int missionToBytes(MissionCommand* commands, uint8_t num, uint8_t* bytes);
/**
 * \~English Calculates a size of array to store mission in raw bytes format.
 * \param[in] commands Array of mission commands.
 * \param[in] num Number of mission commands.
 * \return Returns the size required to store the mission.
 * \~Russian Вычисляет размер массива для хранения миссии в виде массива необработанных байтов.
 * \param[in] commands Массив команд миссии.
 * \param[in] num Количество команд миссии.
 * \return Возвращает размер массива, требуемого длоя хранения миссии.
 */
uint32_t getMissionBytesSize(MissionCommand* commands, uint8_t num);

/**
 * \~English Converts a no-flight areas string received from the ATM server into an array of areas of \ref NoFlightArea type.
 * \param[in] areas String from the ATM server. Expected form is "$ForbiddenZones areas#".
 * \return Returns 1 on successful conversion, 0 otherwise.
 * \~Russian Преобразует строку с бесполетным зонами, полученную от сервера ОРВД, в массив зон типа \ref NoFlightArea.
 * \param[in] areas Строка, пришедшая от сервера ОРВД. Ожидается в виде "$ForbiddenZones зоны#".
 * \return Возвращает 1, если зоны были успешно распознаны, иначе -- 0.
 */
int loadNoFlightAreas(char* areas);
/**
 * \~English Converts a string with changes in no-flight areas received from the ATM server, and updates current no-flight areas array.
 * \param[in] areas String from the ATM server. Expected form is "$ForbiddenZonesDelta changes#".
 * \return Returns 1 on successful conversion, 0 otherwise.
 * \~Russian Преобразует строку с изменениями бесполетных зон, полученную от сервера ОРВД, и обновляет массив бесполетных зон.
 * \param[in] areas Строка, пришедшая от сервера ОРВД. Ожидается в виде "$ForbiddenZonesDelta изменения#".
 * \return Возвращает 1, если изменения зон были успешно распознаны, иначе -- 0.
 */
int updateNoFlightAreas(char* areas);
/**
 * \~English Deletes all no-flight areas.
 * \~Russian Очищает список бесполетных зон.
 */
void deleteNoFlightAreas();
/**
 * \~English Logs current no-flight areas and at the same time prints them to the console.
 * \~Russian Записывает имеющиеся бесполетные зоны в лог и одновременно выводит их в консоль.
 */
void printNoFlightAreas();
/**
 * \~English Returns current no-flight areas.
 * \param[out] num Number of no-flight areas.
 * \return Returns pointer to a no-flight areas array.
 * \~Russian Возвращает текущие бесполетные зоны.
 * \param[out] num Число бесполетных зон.
 * \return Возвращает указатель на массив бесполетных зон.
 */
NoFlightArea* getNoFlightAreas(int &num);

/**
 * \~English Calculates hash of current no-flight areas list.
 * \return Returns pointer to the calculated hash string.
 * \~Russian Вычисляет хэш-значение текущего списка бесполетных зон.
 * \return Возвращает указатель на строку с вычисленным хэшем.
 */
char* getNoFlightAreasHash();
/**
 * \~English Extracts hash from the string received from the ATM server.
 * \param[in] response String from the ATM server. Substring "$ForbiddenZonesHash hash$" is expected.
 * \param[out] hash String to write hash.
 * \param[in] hashLen Length of the string to write hash
 * \~Russian Извлекает хэш из стркои, полученной от сервера ОРВД.
 * \param[in] response Строка, пришедшая от сервера ОРВД. Ожидается наличие подстроки "$ForbiddenZonesHash хэш$".
 * \param[out] hash Строка, куда будет записан хэш.
 * \param[in] hashLen Длина строки, куда будет записан хэш.
 */
void parseNoFlightAreasHash(char* response, char* hash, uint8_t hashLen);

/**
 * \~English Extracts hash from the string received from the ATM server.
 * \param[in] response String from the ATM server. Substring "$Delay delay#" is expected.
 * \return Returns delay until next communication session in seconds.
 * \param[in] hashLen Length of the string to write hash
 * \~Russian Извлекает задержку до следующего сеанса связи из стркои, полученной от сервера ОРВД.
 * \param[in] response Строка, пришедшая от сервера ОРВД. Ожидается наличие подстроки "$Delay задержка#".
 * \return Возвращает задержку до следующего сеанса связи в секундах.
 */
uint32_t parseDelay(char* response);