/**
 * \file
 * \~English \brief Declaration of wrapper methods that send IPC messages to AutopilotConnector component.
 * \~Russian \brief Объявление методов-оберток для отправки IPC-сообщений компоненту AutopilotConnector.
 */

#pragma once

#include <stdint.h>

/**
 * \~English Makes AutopilotConnector component wait for an arm request from the autopilot.
 * \warning Synchronous method. Will not end until AutopilotConnector receives a message from the autopilot.
 * \return Returns 1 on succesful arm request receive, 0 otherwise.
 * \~Russian Переводит компонент AutopilotConnector в режим ожидания запроса на арминг от автопилота.
 * \warning Синхронный метод. Не завершится, пока AutopilotConnector не получит сообщение от автопилота.
 * \return Возвращает 1, если запрос на арминг был получен от автопилота, иначе -- 0.
 */
int waitForArmRequest();
/**
 * \~English Sends a message to the autopilot that allows drone arm.
 * \note Sent message does not force the autopilot to arm the drone.
 * \return Returns 1 on succesful message send, 0 otherwise.
 * \~Russian Передает в автопилот сообщение, разрешающее арминг.
 * \note Переданное сообщение не заставляет автопилот произвести арминг дрона.
 * \return Возвращает 1, если сообщение было успешно передано, иначе -- 0.
 */
int permitArm();
/**
 * \~English Sends a message to the autopilot that forbids drone arm.
 * \note Sent message does not guarantee that the autopilot will no longer try to arm the drone.
 * \return Returns 1 on succesful message send, 0 otherwise.
 * \~Russian Передает в автопилот сообщение, запрещающее арминг.
 * \note Переданное сообщение не гарантирует, что автопилот больше не будет пытаться произвести арминг дрона.
 * \return Возвращает 1, если сообщение было успешно передано, иначе -- 0.
 */
int forbidArm();
/**
 * \~English Sends a message to the autopilot demanding to pause the flight.
 * \return Returns 1 on succesful message send, 0 otherwise.
 * \~Russian Передает в автопилот сообщение с требованием приостановить полет.
 * \return Возвращает 1, если сообщение было успешно передано, иначе -- 0.
 */
int pauseFlight();
/**
 * \~English Sends a message to the autopilot demanding to resume the flight.
 * \return Returns 1 on succesful message send, 0 otherwise.
 * \~Russian Передает в автопилот сообщение с требованием возобновить полет.
 * \return Возвращает 1, если сообщение было успешно передано, иначе -- 0.
 */
int resumeFlight();
/**
 * \~English Sends a message to the autopilot demanding to abort current mission.
 * \return Returns 1 on succesful message send, 0 otherwise.
 * \~Russian Передает в автопилот сообщение с требованием отмены миссии.
 * \return Возвращает 1, если сообщение было успешно передано, иначе -- 0.
 */
int abortMission();
/**
 * \~English Sends a message to the autopilot demanding to change flight speed.
 * \param[in] speed Flight speed in cm/s at which the drone should continue to move.
 * \return Returns 1 on succesful message send, 0 otherwise.
 * \~Russian Передает в автопилот сообщение с требованием изменить скорость полета.
 * \param[in] speed Скорость в см/с, с которой должен продолжить движение дрон.
 * \return Возвращает 1, если сообщение было успешно передано, иначе -- 0.
 */
int changeSpeed(int32_t speed);
/**
 * \~English Sends a message to the autopilot demanding to change the entire flight altitude.
 * \param[in] altitude Altitude in cm (relative to the home point) at which the entire subsequent flight should be performed.
 * \return Returns 1 on succesful message send, 0 otherwise.
 * \~Russian Передает в автопилот сообщение с требованием изменить высоту всего полета.
 * \param[in] altitude Высота в см (относительно точки дома), на которой должен производиться весь последующий полет.
 * \return Возвращает 1, если сообщение было успешно передано, иначе -- 0.
 */
int changeAltitude(int32_t altitude);
/**
 * \~English Sends a message to autopilot demanding to change the current destination point.
 * \param[in] latitude Latitude of the point in degrees * 10^7 at which the drone should start moving.
 * \param[in] longitude Longitude of the point in degrees * 10^7 at which the drone should start moving.
 * \param[in] altitude Altitude of the point in cm at which the drone should start moving.
 * \return Returns 1 on succesful message send, 0 otherwise.
 * \~Russian Передает в автопилот сообщение с требованием изменить текущую точку назначения.
 * \param[in] latitude Широта точки в градусах * 10^7, в которую должен начать двигаться дрон.
 * \param[in] longitude Долгота точки в градусах * 10^7, в которую должен начать двигаться дрон.
 * \param[in] altitude Высота точки в см, в которую должен начать двигаться дрон.
 * \return Возвращает 1, если сообщение было успешно передано, иначе -- 0.
 */
int changeWaypoint(int32_t latitude, int32_t longitude, int32_t altitude);
/**
 * \~English Sends a mission to autopilot demanding to set it as new.
 * \param[in] mission Mission as a raw byte array.
 * \param[in] missionSize Byte arrays size.
 * \return Returns 1 on succesful message send, 0 otherwise.
 * \~Russian Отправляет в автопилот новую миссию с требованием заменить ею текущую.
 * \param[in] mission Миссия в виде массива байтов.
 * \param[in] missionSize Размер массива с миссией.
 * \return Возвращает 1, если сообщение было успешно передано, иначе -- 0.
 */
int setMission(uint8_t* mission, uint32_t missionSize);