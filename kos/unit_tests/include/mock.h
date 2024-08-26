/**
 * \file
 * \~English \brief Declaration of private security module components methods to test and auxiliary methods.
 * \~Russian \brief Объявление приватных тестируемых методов компонентов модуля безопасности и вспомогательных методов.
 */

#include <stdlib.h>

/**
 * \~English Returns last entry that should have been logged.
 * \return Meaningful part of log entry (without date, log level and component name).
 * \note When testing, all log entries go here, and not to Logger component.
 * \~Russian Возвращает последнюю запись, которая должна была быть записана в лог.
 * \return Значимая часть логируемого сообщения (без даты, уровня логи и имени компонента).
 * \note При тестировании все логирование попадает сюда, а не в компонент Logger.
 */
char* getMockLog();
/**
 * \~English Returns mock buzzer mode.
 * \return Returns 1 if mock buzzer is enabled, 0 otherwise.
 * \note When testing, PeripheryController component changes value of mock buzzer.
 * \~Russian Возвращает режим ложного зуммера.
 * \return Возвращает 1, если ложный зуммер включен, иначе -- 0.
 * \note При тестировании компонент PeripheryController управляет возвращаемым здесь ложным значением.
 */
bool getMockBuzzer();
/**
 * \~English Sets mock buzzer mode.
 * \param[in] enable Mock buzzer mode.
 * \~Russian Устанавливает режим ложного зуммера.
 * \param[in] enable Режим ложного зуммера.
 */
void setMockBuzzer(bool enable);

uint8_t hexCharToInt(char c);
void stringToBytes(char* source, uint32_t sourceSize, uint8_t* destination);

void hashToKey(uint8_t* source, uint32_t sourceSize, uint8_t* destination);
void bytesToString(uint8_t* source, char* destination);

int isStopSymbol(char character);
int parseInt(char*& string, int32_t& value, uint32_t numAfterPoint);
int parseCommands(char* str);

void buzz();