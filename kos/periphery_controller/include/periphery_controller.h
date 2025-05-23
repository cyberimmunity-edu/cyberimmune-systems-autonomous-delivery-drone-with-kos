/**
 * \file
 * \~English
 * \brief Declaration of methods for peripherals control.
 * \details The file contains declaration of methods,
 * that control peripheral devices via GPIO interface.
 *
 * \~Russian
 * \brief Объявление методов для работы с периферийными устройствами.
 * \details В файле объявлены публичныe методы, необходимые для управления
 * периферийными устройствами через интерфейс GPIO.
 */

#pragma once

#include "../../shared/include/ipc_messages_logger.h"

/**
 * \~English Initializes softwate and hardware components required
 * for peripherals control.
 * \return Returns 1 on successful initialization, 0 otherwise.
 * \~Russian Производит инициализацию программных и аппаратных компонентов, необходимых
 * для управления периферийными устройствами.
 * \return Возвращает 1 при успешной инициализации, 0 -- иначе.
 */
int initPeripheryController();
/**
 * \~English Checks communication with all peripherals.
 * \return Returns 1, if peripherals are available, 0 otherwise.
 * \~Russian Проверяет наличие связи со всеми периферийными устройствами.
 * \return Возвращает 1, если устройства доступны, 0 -- иначе.
 */
int initGpioPins();

/**
 * \~English Reports the current mode of drone motors power supply.
 * \return Returns 1, if motors is supplied with power, 0 otherwise.
 * \~Russian Сообщает текущий режим подачи питания на двигатели дрона.
 * \return Возвращает 1, если питание подано, 0 -- иначе.
 */
bool isKillSwitchEnabled();
/**
 * \~English Sets the buzzer mode to a given one.
 * \param[in] enable New buzzer mode.
 * \return Returns 1 on successful mode set, 0 otherwise.
 * \~Russian Устанавливает режим работы зуммера на поданный.
 * \param[in] enable Новый режим работы зуммера.
 * \return Возвращает 1 при успешной установке режима, 0 -- иначе.
 */
int setBuzzer(bool enable);

/**
 * \~English Turns on the buzzer and then starts a separate process that will turn off
 * the buzzer after 2 seconds. The buzzer will not turn on if it has already been turned on with this method.
 * \return Returns 1 on successful turn on, 0 otherwise.
 * \~Russian Включает зуммер, после чего запускает отдельный процесс, который отключит зуммер
 * через 2 секунды. Зуммер не включится, если он уже был включен этим методом.
 * \return Возвращает 1, если зуммер был успешно включен, иначе -- 0.
 */
int startBuzzer();
/**
 * \~English Switches the mode of drone motors power supply.
 * \param[in] enable Power supply mode.
 * \return Returns 1 on successful mode set, 0 otherwise.
 * \~Russian Переключает режим подачи питания на двигатели.
 * \param[in] enable Режим подачи питания.
 * \return Возвращает 1, если режим был успешно установлен, иначе -- 0.
 */
int setKillSwitch(bool enable);
/**
 * \~English Switches the mode of cargo drop motor power supply.
 * \param[in] enable Power supply mode.
 * \return Returns 1 on successful mode set, 0 otherwise.
 * \~Russian Переключает режим подачb питания на мотор сброса груза.
 * \param[in] enable Режим подачи питания.
 * \return Возвращает 1, если режим был успешно установлен, иначе -- 0.
 */
int setCargoLock(bool enable);