/**
 * \file
 * \~English
 * \brief Declaration of methods for drone position monitoring.
 * \details The file contains declaration of methods,
 * that receive, store, update and transmit current drone location.
 *
 * \~Russian
 * \brief Объявление методов для работы с местоположением дрона.
 * \details В файле объявлены публичныe методы, необходимые для
 * получения, хранения, обновления и передачи информации о текущем
 * местоположении дрона.
 */

#pragma once

#include "../../shared/include/ipc_messages_logger.h"
#include <stdint.h>

/**
 * \~English Initializes softwate and hardware components required
 * for data receive from drone simulator or GNSS module and barometer.
 * \return Returns 1 on successful initialization, 0 otherwise.
 * \~Russian Производит инициализацию программных и аппаратных компонентов, необходимых
 * для получения данных от симулятора дрона или модуля GNSS и барометра.
 * \return Возвращает 1 при успешной инициализации, 0 -- иначе.
 */
int initNavigationSystem();
/**
 * \~English Establishes communication and communication protocol with modules
 * that will be a source of drone position.
 * \return Returns 1 on successful communication establishment, 0 otherwise.
 * \~Russian Устанавливает связь и протокол общения с модулями, которые будут
 * источником информации о местоположении дрона.
 * \return Возвращает 1 при успешной установке связи, 0 -- иначе.
 */
int initSensors();

/**
 * \~English Reports whether the security module has begun to receive up-to-date correct drone position.
 * \return Returns 1 if data was received, 0 otherwise.
 * \note GNSS module, barometer and SITL firmware do not start up-to-date data send instantly.
 * Prior to full initialization, they send zero-data. To work with drone postion, the arrival of
 * non-zero data must be waited. In case of malfunction, these modules also send zeroed data.
 * \~Russian Сообщает, начал ли модуль безопасности получать актуальные корректные
 * данные о местоположении дрона.
 * \return Возвращает 1, если данные были получены, 0 -- иначе.
 * \note Модуль GNSS, барометр и SITL-прошивка начинают отправку актуальных данных не мгновенно.
 * До полной инициализации они отправляют данные, состоящие из нулей. Для работы с местоположением,
 * необходимо дождаться начала поступления ненулевых данных. В случае неисправности, эти модули
 * также отправляют данные, представляющие из себя нули.
 */
bool hasPosition();

/**
 * \~English Procedure that receives drone position (from a SITL firmware or GNSS module) and updates the current location
 * with the received data using \ref setInfo, \ref setAltitude, and \ref setCoords. It is assumed that this procedure
 * is looped and is performed in a parallel thread.
 * \~Russian Процедура, выполняющая получение данных о местоположении дрона (от SITL-прошивки или модуля GNSS), и обновление
 * текущего местоположения полученными данными при помощи \ref setInfo, \ref setAltitude и \ref setCoords. Предполагается,
 * что данная процедура выполняется циклически в параллельной нити.
 */
void getSensors();
/**
 * \~English Procedure that transmits drone position to the ATM server. Signs and transmits messages through
 * CredentialManager and ServerConnector components respectively. It is assumed that this procedure
 * is looped and is performed in a parallel thread.
 * \~Russian Процедура, выполняющая передачу данных о местоположении дрона на сервер ОРВД. Подписывает сообщения и передает
 * их через компоненты CredentialManager и ServerConnector соответственно. Предполагается,
 * что данная процедура выполняется циклически в параллельной нити.
 */
void sendCoords();

/**
 * \~English Updates information about positioning precision with a new data.
 * \param[in] dop Current DOP value (dilution of precision).
 * \param[in] sats Current number of observed satellites.
 * \~Russian Обновляет информацию о точности позиционирования новыми данными.
 * \param[in] dop Текущее значение DOP (снижение точности).
 * \param[in] sats Текущее количество наблюдаемых спутников.
 */
void setInfo(float dop, int32_t sats);
/**
 * \~English Returns information about positioning precision.
 * \param[out] dop DOP value (dilution of precision).
 * \param[out] sats The number of observed satellites. Varies from 0 to 12.
 * \return Returns 1 on successful information receive, 0 otherwise.
 * \~Russian Возвращает информацию о точности позиционирования.
 * \param[out] dop Значение DOP (снижение точности).
 * \param[out] sats Количество наблюдаемых спутников, принимает значения от 0 до 12.
 * \return Возвращает 1, если информация была успешно получена, иначе -- 0.
 */
int getInfo(float& dop, int32_t &sats);

/**
 * \~English Updates current absolute altitude of the drone.
 * \param[in] altitude Current absolute altitude in cm.
 * \~Russian Обновляет значение текущей абсолютной высоты дрона.
 * \param[in] altitude Текущая абсолютная высота в см.
 */
void setAltitude(int32_t altitude);
/**
 * \~English Updates current latitude and longitude of the drone.
 * \param[in] latitude Current latitude in degrees * 10^7.
 * \param[in] longitude Current longitude in degrees * 10^7.
 * \~Russian Обновляет значения текущих широты и высоты дрона.
 * \param[in] latitude Текущая широта в градусах * 10^7.
 * \param[in] longitude Текущая долгота в градусах * 10^7.
 */
void setCoords(int32_t latitude, int32_t longitude);
/**
 * \~English Returns current coordinates of the drone.
 * \param[out] latitude Current latitude in degrees * 10^7.
 * \param[out] longitude Current longitude in degrees * 10^7.
 * \param[out] altitude Current absolute altitude in cm.
 * \return Returns 1 on successful coordinates receive, 0 otherwise.
 * \~Russian Возвращает текущие координаты дрона.
 * \param[out] latitude Текущая широта в градусах * 10^7.
 * \param[out] longitude Текущая долгота в градусах * 10^7.
 * \param[out] altitude Текущая абсолютная высота в см.
 * \return Возвращает 1, если координаты были успешно получены, иначе -- 0.
 */
int getPosition(int32_t &latitude, int32_t &longitude, int32_t &altitude);

/**
 * \~English Updates current speed of the drone.
 * \param[in] speed Current speed in m/s.
 * \~Russian Обновляет значение текущей скорости дрона.
 * \param[in] speed Текущая скорость в м/с.
 */
void setSpeed(float speed);
/**
 * \~English Returns current speed of the drone.
 * \param[out] speed Current speed in m/s.
 * \return Returns 1 on successful speed receive, 0 otherwise.
 * \~Russian Возвращает текущую скорость дрона.
 * \param[out] speed Текущая скорость в м/с.
 * \return Возвращает 1, если скорость была успешно получена, иначе -- 0.
 */
int getSpeed(float &speed);