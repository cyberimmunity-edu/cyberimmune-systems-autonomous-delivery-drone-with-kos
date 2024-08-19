/**
 * \file
 * \~English
 * \brief Declaration of methods for RSA signatures management.
 * \details The file contains declaration of methods,
 * that allow signing security module messages and checking
 * the authenticity of the ARM server messages.
 * These methods rely on MbedTLS library.
 *
 * \~Russian
 * \brief Объявление методов для работы с RSA-подписями.
 * \details В файле объявлены публичныe методы, необходимые
 * для подписи сообщений модуля безопасности, а также для проверки
 * аутентичности сообщений сервера ОРВД.
 * Эти методы полагаются на библиотеку MbedTLS.
 */

#pragma once

#include "../../shared/include/ipc_messages_logger.h"
#include <stdint.h>

/**
 * \~English Generates 3 parts of the RSA key that
 * will be used by the security module to sign messages.
 * \return Returns 1 on successful generation. 0 otherwise.
 * \~Russian Генерирует 3 части RSA-ключа, который будет использоваться модулем
 * безопасности для дальнейшей подписи сообщений.
 * \return Возвращает 1 при успешной генерации, иначе -- 0.
 */
int generateRsaKey();
/**
 * \~English Sets the private part of the RSA key for future message signing
 * and the public part for exchange with the ATM server.
 * \param[in] N RSA key shared part, that belongs both to public and open parts.
 * \param[in] D RSA key private part, known only by owner.
 * \param[in] n RSA key shared part, that belongs both to public and open parts.
 * Differs from N only in format.
 * \param[in] e RSA key public part, knwon by anyone who wants to check authenticity of the message.
 * \param[in] nLen Length of RSA key n part.
 * \param[in] eLen Length of RSA key e part.
 * \return Returns 1 on successful key parts set, 0 otherwise.
 * \~Russian Устанавливает закрытую часть RSA-ключа для использования в дальнейшей подписи сообщений и
 * открытую часть для обмена с сервером ОРВД.
 * \param[in] N Общая часть RSA-ключа, входящая и в закрытую, и в открытую части.
 * \param[in] D Закрытая часть RSA-ключа, имеющаяся только у владельца.
 * \param[in] n Общая часть RSA-ключа, входящая и в закрытую, и в открытую части.
 * Отличается от N только форматом.
 * \param[in] e Открытая часть RSA-ключа, имеющаяся у любого, кто хочет проверит аутентичность сообщения.
 * \param[in] nLen Длина части n RSA-ключа.
 * \param[in] eLen Длина части e RSA-ключа.
 * \return Возвращает 1, если все части ключа успешно установлены, иначе -- 0.
 */
int loadRsaKey(uint8_t* N, uint8_t* D, char* n, char* e, uint32_t nLen, uint32_t eLen);
/**
 * \~English Exchanges RSA key public parts with the ATM server.
 * \return Returns 1 on successful exchange, 0 otherwise.
 * \note The keys are exchanged with a request to the ATM server through ServerConnector component.
 * \~Russian Производит обмен открытыми частями RSA-ключей с сервером ОРВД.
 * \return Возвращает 1 при успешном обмене, 0 -- иначе.
 * \note Обмен ключами происходит при помощи запроса на сервер ОРВД через компонент ServerConnector.
 */
int shareRsaKey();

/**
 * \~English Attempts to read stored RSA key. If file "rsa" with key is found, sets the key parts
 * with \ref loadRsaKey, otherwise generates a new key with \ref generateRsaKey and writes it to file "rsa".
 * \return Returns 1 on successful key get, 0 otherwise.
 * \~Russian Производит попытку прочесть сохраненный RSA-ключ. При нахождении файла "rsa" с ключом,
 * устанавливает его части с помощью \ref loadRsaKey, иначе -- генерирует новый ключ с помощью \ref generateRsaKey
 * и записывает его в файл "rsa".
 * \return Возвращает 1, если ключ был успешно получен, иначе -- 0.
 */
int getRsaKey();
/**
 * \~English Sets a public part of the ATM server RSA key for future check of received messages authenticity.
 * \param[in] key String with a public part of key. Requires "$Key: n-key_part e-key_part" format.
 * \return Returns 1 on successful key set, 0 otherwise.
 * \~Russian Устанавливает открытую часть RSA-ключа сервера ОРВД для дальнейшего
 * использования в проверке аутентичности получаемых сообщений.
 * \param[in] key Строка с открытой частью ключа. Требуемый формат строки: "$Key: n-часть_ключа e-часть_ключа".
 * \return Возвращает 1, если ключ был успешно установлен, иначе -- 0.
 */
int setRsaKey(char* key);

/**
 * \~English Computes an RSA signature of a given message.
 * \param[in] message Message to sign.
 * \param[out] signature Computed signature. Has 256 bytes size.
 * \return Returns 1 on succesful signature compute, 0 otherwise.
 * \~Russian Вычисляет RSA-подпись поданного сообщения.
 * \param[in] message Сообщение, подпись для которого необходимо вычислить.
 * \param[out] signature Вычисленная подпись. Занимает 256 байт.
 * \return Возвращает 1, если подпись была успешно вычислена, иначе -- 0.
 */
int getMessageSignature(char* message, char* sign);
/**
 * \~English Checks the authenticity of an ATM server message.
 * \param[in] message Message to check authenticity. Requires "message#signature" format.
 * \param[out] authenticity Message authenticity check result.
 * \return Returns 1 on authenticity check (not confirmation), 0 otherwise.
 * \~Russian Проверяет аутентичность сообщения, полученного от сервера ОРВД.
 * \param[in] message Сообщение, аутентичность которого необходимо проверить. Требуемый формат сообщения: "сообщение#подпись".
 * \param[out] authenticity Результат проверки аутентичности сообщения.
 * \return Возвращает 1, если аутентичность была проверена (но не обязательно подтверждена), иначе -- 0.
 */
int checkMessageSignature(char* message, uint8_t &correct);

/**
 * \~English Returns n-part of security module RSA key.
 * \return N-part of key as hexadecimal string.
 * \~Russian Возвращает n-часть RSA-ключа модуля безопасности.
 * \return N-часть ключа в виде 16-ричной строки.
 */
char* getKeyN();
/**
 * \~English Returns e-part of security module RSA key.
 * \return E-part of key as hexadecimal string.
 * \~Russian Возвращает e-часть RSA-ключа модуля безопасности.
 * \return E-часть ключа в виде 16-ричной строки.
 */
char* getKeyE();
/**
 * \~English Returns d-part of security module RSA key.
 * \return D-part of key as hexadecimal string.
 * \~Russian Возвращает d-часть RSA-ключа модуля безопасности.
 * \return D-часть ключа в виде 16-ричной строки.
 */
char* getKeyD();