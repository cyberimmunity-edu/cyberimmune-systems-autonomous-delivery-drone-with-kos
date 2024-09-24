/**
 * \file
 * \~English \brief Declaration of wrapper methods that send IPC messages to CredentialManager component.
 * \~Russian \brief Объявление методов-оберток для отправки IPC-сообщений компоненту CredentialManager.
 */

#pragma once

#include <stdint.h>

/**
 * \~English Computes an RSA signature of the message.
 * \param[in] message Message to sign.
 * \param[out] signature Computed signature. Expected to be at least 256 bytes.
 * \param[in] signatureSize Size of output signature buffer.
 * \return Returns 1 on succesful signature compute, 0 otherwise.
 * \~Russian Вычисляет RSA-подпись сообщения.
 * \param[in] message Сообщение, подпись для которого необходимо вычислить.
 * \param[out] signature Вычисленная подпись. Ожидается буфер не меньше 256 байт.
 * \param[in] signatureSize Размер буфера, куда записывается вычисленная подпись.
 * \return Возвращает 1, если подпись была успешно вычислена, иначе -- 0.
 */
int signMessage(char* message, char* signature, uint32_t signatureSize);
/**
 * \~English Checks the authenticity of a message received from the ATM server.
 * \param[in] message Message to check authenticity. Requires "message#signature" format.
 * \param[out] authenticity Message authenticity check result.
 * \return Returns 1 on authenticity check (not confirmation), 0 otherwise.
 * \~Russian Проверяет аутентичность сообщения, полученного от сервера ОРВД.
 * \param[in] message Сообщение, аутентичность которого необходимо проверить. Требуемый формат сообщения: "сообщение#подпись".
 * \param[out] authenticity Результат проверки аутентичности сообщения.
 * \return Возвращает 1, если аутентичность была проверена (но не обязательно подтверждена), иначе -- 0.
 */
int checkSignature(char* message, uint8_t &authenticity);