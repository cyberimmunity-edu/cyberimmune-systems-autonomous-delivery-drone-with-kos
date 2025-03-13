/**
 * \file
 * \~English
 * \brief Declaration of methods for autopilot communication.
 * \details The file contains declaration of methods,
 * that provide interaction between the security module and a
 * compatible ArduPilot firmware.
 *
 * \~Russian
 * \brief Объявление методов для взаимодействия с автопилотом.
 * \details В файле объявлены публичныe методы, необходимые
 * для взаимодействия между модулем безопасности и
 * совместимой прошивкой ArduPilot.
 */

#pragma once

#include "../../shared/include/ipc_messages_logger.h"
#include <stdint.h>
#include <unistd.h>

/** \cond */
#define AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE 4

static const uint8_t AutopilotCommandMessageHead[AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE] = { 0x7a, 0xfe, 0xf0, 0x0d };
/** \endcond */

/**
 * \~English Control messages types used for communication between
 * the security module and an autopilot.
 * \~Russian Типы управляющих сообщений, используемых
 * при взаимодействии между модулем безопасности и автопилотом.
 */
enum AutopilotCommand : uint8_t {
    /**
     * \~English Auxiliary type. Transmitted in both directions only as an error signal.
     * The default type for an empty undefined message.
     * \~Russian Вспомогательный тип. Передается в обе стороны только как сигнал об ошибке.
     * Тип по умолчанию для пустого неопределенного сообщения.
     */
    ERROR = 0x00,
    /**
     * \~English Arm request. Transmitted from the autopilot to the security module on arm attempt.
     * \~Russian Запрос на арминг. Передается от автопилота в модуль безопасности при попытке выполнить арминг.
     */
    ArmRequest = 0x3A,
    /**
     * \~English Arm approval. Transmitted from the security module to the autopilot in response to an allowed arm request.
     * \~Russian Одобрение арминга. Передается от модуля безопасности в автопилот в ответ на запрос при разрешении арминга.
     */
    ArmPermit = 0xA5,
    /**
     * \~English Arm denial. Transmitted from the security module to the autopilot in response to a forbidden arm request.
     * \~Russian Отказ в арминге. Передается от модуля безопасности в автопилот в ответ на запрос при запрете арминга.
     */
    ArmForbid = 0xE4,
    /**
     * \~English Flight pause requirement. Transmitted from the security module to the autopilot during an active flight.
     * \~Russian Требование на приостановку полета. Передается от модуля безопасности в автопилот при активном полете.
     */
    PauseFlight = 0xAB,
    /**
     * \~English Flight resume requirement. Transmitted from the security module to the autopilot during a paused flight.
     * \~Russian Требование на возобновление полета. Передается от модуля безопасности в автопилот при приостановленном полете.
     */
    ResumeFlight = 0x47,
    /**
     * \~English A requirement to change current destination. Transmitted from the security module to the autopilot during a paused flight.
     * \~Russian Требование на изменение текущей точки назначения. Передается от модуля безопасности в автопилот при активном полете.
     */
    ChangeWaypoint = 0x10,
    /**
     * \~English A requirement to change current drone speed. Transmitted from the security module to the autopilot during a paused flight.
     * \~Russian Требование на изменение скорости дрона. Передается от модуля безопасности в автопилот при активном полете.
     */
    ChangeSpeed = 0xEE,
    /**
     * \~English A requirement to change current flight altitude. Transmitted from the security module to the autopilot during a paused flight.
     * \~Russian Требование на изменение высоты полета. Передается от модуля безопасности в автопилот при активном полете.
     */
    ChangeAltitude = 0xA1,
    /**
     * \~English Abort mission requirement. Transmitted from the security module to the autopilot during an active flight.
     * When mission is aborted, the vehicle holds its current position in the air.
     * \~Russian Требование на отмену миссии. Передается от модуля безопасности в автопилот при активном полете.
     * При отмене миссии, квадрокоптер зависает в воздухе.
     */
    AbortMission = 0x21,
    /**
     * \~English A requirement to set passed mission.
     * \~Russian Требование изменения миссии на переданную.
     */
    SetMission = 0x42
};

/**
 * \~English Structure describing a main body of the message used for communication
 * between the security module and an autopilot.
 * \~Russian Структура, описывающая основное тело сообщения, использующегося
 * для коммуникации между модулем безопасности и автопилотом.
 */
struct AutopilotCommandMessage {
    /**
     * \~English Fixed header bytes that indicate the start of a new message.
     * \~Russian Фиксированные заголовочные байты, указывающие на начало нового сообщения.
     */
    uint8_t head[AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE];
    /**
     * \~English Message type. For the types used, see \ref AutopilotCommand.
     * \~Russian Тип сообщения. Используемые типы см. в \ref AutopilotCommand.
     */
    AutopilotCommand command;

    /**
     * \~English Default constructor. Sets the correct header bytes and ERROR command type.
     * \~Russian Конструктор по умолчанию. Устанавливает корректные заголовочные байты и тип команды ERROR.
     */
    AutopilotCommandMessage() {
        for (int i = 0; i < AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE; i++)
            head[i] = 0;
        command = AutopilotCommand::ERROR;
    }

    /**
     * \~English Constructor that sets the correct header bytes and given type command.
     * \param[in] _command Created message type.
     * \~Russian Конструктор, устанавливающий корректные заголовочные байты и команду поданного типа.
     * \param[in] _command Тип создаваемого сообщения.
     */
    AutopilotCommandMessage(AutopilotCommand _command) {
        for (int i = 0; i < AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE; i++)
            head[i] = AutopilotCommandMessageHead[i];
        command = _command;
    }
};

/**
 * \~English Initializes softwate and hardware components required
 * for the interaction between the security module and an autopilot.
 * \return Returns 1 on successful initialization, 0 otherwise.
 * \~Russian Производит инициализацию программных и аппаратных компонентов, необходимых
 * для взаимодействия между автопилотом и модулем безопасности.
 * \return Возвращает 1 при успешной инициализации, 0 -- иначе.
 */
int initAutopilotConnector();
/**
 * \~English Attempts to establish communication with the autopilot.
 * \return Returns 1 on successful attempt, 0 otherwise.
 * \~Russian Производит попытку установить связь с автопилотом.
 * \return Возвращает 1 при успешном установлении соединения, 0 -- иначе.
 */
int initConnection();

/**
 * \~English Waits for a message from the autopilot.
 * \warning Synchronous method. Ends only when a message is received from the autopilot.
 * \param[out] command Received message type.
 * \return Returns 1 on successful message receive, 0 otherwise.
 * \~Russian Ожидает сообщения от автопилота.
 * \warning Синхронный метод. Завершится, лишь когда сообщение от автопилота будет получено.
 * \param[out] command Тип полученного сообщения.
 * \return Возвращает 1 при успешном получении сообщения, 0 -- иначе.
 */
int getAutopilotCommand(uint8_t& command);
/**
 * \~English Sends raw bytes to the autopilot.
 * \param[in] bytes Pointer to byte array.
 * \param[in] size Size of byte array to send.
 * \return Returns 1 on successful send, 0 otherwise.
 * \~Russian Отправляет массив байтов автопилоту.
 * \param[in] bytes Указатель на массив байтов.
 * \param[in] size Размер отправляемого массива.
 * \return Возвращает 1 при успешном получении сообщения, 0 -- иначе.
 */
int sendAutopilotBytes(uint8_t* bytes, ssize_t size);

/**
 * \~English Sends a message to the autopilot. A version without additional data sent.
 * \param[in] command Sent message type.
 * \return Returns 1 on successful message send, 0 otherwise.
 * \note Is expected to send ArmPermit, ArmForbid, PauseFlight and ResumeFlight commands.
 * \~Russian Отправляет автопилоту сообщение. Вариант без дополнительных данных.
 * \param[in] command Тип отправляемого сообщения.
 * \return Возвращает 1 при успешной отправке сообщения, 0 -- иначе.
 * \note Предполагается для отправки команд ArmPermit, ArmForbid, PauseFlight и ResumeFlight.
 */
int sendAutopilotCommand(AutopilotCommand command);
/**
 * \~English Sends a message to the autopilot. A version with one additional transmitted value.
 * \param[in] command Sent message type.
 * \param[in] value Sent value.
 * \return Returns 1 on successful message send, 0 otherwise.
 * \note Is expected to send ChangeSpeed or ChangeAltitude commands and the desired speed or altitude value.
 * \~Russian Отправляет автопилоту сообщение. Вариант с одним дополнительным передаваемым значением.
 * \param[in] command Тип отправляемого сообщения.
 * \param[in] value Отправляемое значение.
 * \return Возвращает 1 при успешной отправке сообщения, 0 -- иначе.
 * \note Предполагается для отправки команд ChangeSpeed или ChangeAltitude и требуемом значении
 * скорости или высоты.
 */
int sendAutopilotCommand(AutopilotCommand command, int32_t value);
/**
 * \~English Sends a message to the autopilot. A version with three additional transmitted values.
 * \param[in] command Sent message type.
 * \param[in] valueFirst First sent value.
 * \param[in] valueSecond Second sent value.
 * \param[in] valueThird Third sent value.
 * \return Returns 1 on successful message send, 0 otherwise.
 * \note Is expected to send ChangeWaypoint command and a latitude, a longitude, and an altitude
 * of the desired destination point.
 * \~Russian Отправляет автопилоту сообщение. Вариант с тремя дополнительными передаваемыми значениями.
 * \param[in] command Тип отправляемого сообщения.
 * \param[in] valueFirst Первое отправляемое значение.
 * \param[in] valueSecond Первое отправляемое значение.
 * \param[in] valueThird Первое отправляемое значение.
 * \return Возвращает 1 при успешной отправке сообщения, 0 -- иначе.
 * \note Предполагается для отправки команды ChangeWaypoint и широты, долготы и высоты
 * требуемой точки назначения.
 */
int sendAutopilotCommand(AutopilotCommand command, int32_t valueFirst, int32_t valueSecond, int32_t valueThird);
/**
 * \~English Sends a message to the autopilot. A version with raw bytes.
 * \param[in] command Sent message type.
 * \param[in] rawBytes Raw byte array.
 * \param[in] byteSize Size of byte array.
 * \note Is expected to send SetMission command and a new mission as raw bytes.
 * \~Russian Отправляет автопилоту сообщение. Вариант с массивом байтов.
 * \param[in] command Тип отправляемого сообщения.
 * \param[in] rawBytes Массив необработанных байтов.
 * \param[in] byteSize Размер байтового массива.
 * \return Возвращает 1 при успешной отправке сообщения, 0 -- иначе.
 * \note Предполагается для отправки команды SetMission и новой миссии, закодированной в байтах.
 */
int sendAutopilotCommand(AutopilotCommand command, uint8_t* rawBytes, int32_t byteSize);