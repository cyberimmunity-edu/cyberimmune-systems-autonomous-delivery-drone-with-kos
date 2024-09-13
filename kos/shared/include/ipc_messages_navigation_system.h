/**
 * \file
 * \~English \brief Declaration of wrapper methods that send IPC messages to NavigationSystem component.
 * \~Russian \brief Объявление методов-оберток для отправки IPC-сообщений компоненту NavigationSystem.
 */

#pragma once

#include <stdint.h>

/**
 * \~English Returns current coordinates of the drone.
 * \param[out] latitude Current latitude in degrees * 10^7.
 * \param[out] longitude Current longitude in degrees * 10^7.
 * \param[out] altitude Current altitude in cm (absolute value).
 * \return Returns 1 on successful coordinates receive, 0 otherwise.
 * \~Russian Возвращает текущие координаты дрона.
 * \param[out] latitude Текущая широта в градусах * 10^7.
 * \param[out] longitude Текущая долгота в градусах * 10^7.
 * \param[out] altitude Текущая высота в см (абсолютное значение).
 * \return Возвращает 1, если координаты были успешно получены, иначе -- 0.
 */
int getCoords(int32_t &latitude, int32_t &longitude, int32_t &altitude);
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
int getGpsInfo(float& dop, int32_t &sats);
/**
 * \~English Returns estimated speed of the drone.
 * \param[out] speed Estimated speed in m/s.
 * \return Returns 1 on successful speed receive, 0 otherwise.
 * \~Russian Возвращает оцененную скорость дрона.
 * \param[out] speed Оцененная скорость в м/с.
 * \return Возвращает 1, если скорость была успешно получена, иначе -- 0.
 */
int getEstimatedSpeed(float& speed);