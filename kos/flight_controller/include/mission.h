/**
 * \file
 * \~English \brief Declaration of methods for flight mission management.
 * \~Russian \brief Объявление методов для работы с полетной миссией.
 */

#pragma once
#include <stdint.h>

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
 * \~English Converts a mission received from the ATM server into an array of commands of \ref MissionCommand type.
 * \param [in] response Mission from the ATM server. Expected form is "$FlightMission mission#".
 * \return Returns 1 on successful conversion, 0 otherwise.
 * \~Russian Преобразует миссию, полученную от сервера ОРВД, в массив команд типа \ref MissionCommand.
 * \param[in] response Миссия, пришедшая от сервера ОРВД. Ожидается в виде "$FlightMission миссия#".
 * \return Возвращает 1, если миссия была успешно распознана, иначе -- 0.
 */
int parseMission(char* response);
/**
 * \~English Logs an existing mission and at the same time prints it to the console.
 * \~Russian Записывает имеющуюся миссию в лог и одновременно выводит ее в консоль.
 */
void printMission();