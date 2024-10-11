/**
 * \file
 * \~English
 * \brief Implementation of methods for simulated drone peripherals control.
 * \details The file contains implementation of methods, that simulate peripherals
 * control via socket communication with a compatible ArduPilot SITL firmware.
 *
 * \~Russian
 * \brief Реализация методов для работы с симулятором периферии дрона.
 * \details В файле реализованы методы, симулирующие управление периферией дрона
 * через взаимодействие с совместимой SITL-прошивкой ArduPilot через сокет.
 */

#include "../include/periphery_controller.h"

#include <kos_net.h>

#include <stdio.h>

/** \cond */
#define SIM_PERIPHERY_MESSAGE_HEAD_SIZE 4

static const uint8_t SimPeripheryMessageHead[SIM_PERIPHERY_MESSAGE_HEAD_SIZE] = { 0x06, 0x66, 0xbe, 0xa7 };
/** \endcond */

/**
 * \~English Control messages types used to simulate
 * peripherals power supply mod change.
 * \~Russian Типы управляющих сообщений, используемых для симуляции
 * изменения режима подачи питания на периферийные устройства.
 */
enum SimPeripheryCommand : uint8_t {
    /**
     * \~English Auxiliary type. Signals an error.
     * \~Russian Вспомогательный тип. Сигнализирует о возникновении ошибки.
     */
    ERROR = 0x00,
    /**
     * \~English Drone motors are energized. On receive SITL firmware disables software restriction on arming.
     * \~Russian Двигатели запитаны. При получении SITL-прошивка снимает программный запрет на арминг.
     */
    MotorPermit,
    /**
     * \~English Drone motors are de-energized. On receive SITL firmware enables software restriction on arming.
     * \~Russian Двигатели обесточены. При получении SITL-прошивка включает программный запрет на арминг.
     */
    MotorForbid,
    /**
     * \~English Cargo drop motor is energized. On receive SITL firmware disables software restriction on cargo drop.
     * \~Russian Мотор сброса груза запитан. При получении SITL-прошивка снимает программный запрет на сброс груза.
     */
    CargoPermit,
    /**
     * \~English Cargo drop motor is de-energized. On receive SITL firmware enables software restriction on cargo drop.
     * \~Russian Мотор сброса груза обесточен. При получении SITL-прошивка включает программный запрет на сброс груза.
     */
    CargoForbid
};

/**
 * \~English A structure describing a message for SITL firmware
 * to simulate peripherals power supply mod change.
 * transmit current drone position.
 * \~Russian Структура, описывающая сообщение для SITL-прошивки
 * для симуляции изменения режима питания периферийных устройств.
 */
struct SimPeripheryMessage {
    /**
     * \~English Fixed header bytes that indicate the start of a new message.
     * \~Russian Фиксированные заголовочные байты, указывающие на начало нового сообщения.
     */
    uint8_t head[SIM_PERIPHERY_MESSAGE_HEAD_SIZE];
    /**
     * \~English Message type. For the types used, see \ref SimPeripheryCommand.
     * \~Russian Тип сообщения. Используемые типы см. в \ref SimPeripheryCommand.
     */
    SimPeripheryCommand command;
    /**
     * \~English Meaningless bytes. Needed to fill the size of the structure to a multiple of 4 bytes.
     * \~Russian Байты, не имеющие смысловой нагрузки. Нужны для заполнения размера структуры до
     * кратного 4 байтам.
     */
    uint8_t filler[3];

    /**
     * \~English Constructor that sets the correct header bytes and given type command.
     * \param[in] cmd Created message type.
     * \~Russian Конструктор, устанавливающий корректные заголовочные байты и команду поданного типа.
     * \param[in] cmd Тип создаваемого сообщения.
     */
    SimPeripheryMessage(SimPeripheryCommand cmd) {
        for (int i = 0; i < SIM_PERIPHERY_MESSAGE_HEAD_SIZE; i++)
            head[i] = SimPeripheryMessageHead[i];
        command = cmd;
    }
};

/** \cond */
int peripherySocket = NULL;
uint16_t peripheryPort = 5767;

bool killSwitchEnabled;
/** \endcond */

int initPeripheryController() {
    if (!wait_for_network()) {
        logEntry("Connection to network has failed", ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }

    return 1;
}

int initGpioPins() {
    peripherySocket = NULL;

    if ((peripherySocket = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        logEntry("Failed to create socket", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    struct sockaddr_in address = { 0 };
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(SIMULATOR_IP);
    address.sin_port = htons(peripheryPort);

    if (connect(peripherySocket, (struct sockaddr*)&address, sizeof(address)) != 0) {
        char logBuffer[256] = {0};
        snprintf(logBuffer, 256, "Connection to %s:%d has failed", SIMULATOR_IP, peripheryPort);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    return 1;
}

bool isKillSwitchEnabled() {
    return killSwitchEnabled;
}

int setBuzzer(bool enable) {
    char logBuffer[256] = {0};
    snprintf(logBuffer, 256, "Buzzer is %s", enable ? "enabled" : "disabled");
    logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_INFO);

    return 1;
}

int setKillSwitch(bool enable) {
    SimPeripheryMessage message = SimPeripheryMessage(enable ? SimPeripheryCommand::MotorPermit : SimPeripheryCommand::MotorForbid);
    write(peripherySocket, &message, sizeof(SimPeripheryMessage));
    killSwitchEnabled = enable;

    return 1;
}

int setCargoLock(bool enable) {
    SimPeripheryMessage message = SimPeripheryMessage(enable ? SimPeripheryCommand::CargoPermit : SimPeripheryCommand::CargoForbid);
    write(peripherySocket, &message, sizeof(SimPeripheryMessage));

    return 1;
}