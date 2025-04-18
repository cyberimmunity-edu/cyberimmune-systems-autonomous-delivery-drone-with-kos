/**
 * \file
 * \~English
 * \brief Implementation of methods for obtaining drone position.
 * \details The file contains implementation of methods, that obtain current drone position
 * from a GNSS module via UART interface and from a barometer via I2C interface.
 *
 * \~Russian
 * \brief Реализация методов для получения данных о местоположении дрона.
 * \details В файле реализованы методы, для получения информации о текущем местоположении
 * дрона от модуля GNSS через интерфейс UART и через интерфейс I2C -- от барометра.
 */

#include "../include/navigation_system.h"
#include "../../shared/include/ipc_messages_initialization.h"

#include <coresrv/hal/hal_api.h>
#include <rtl/retcode_hr.h>
#include <uart/uart.h>
#include <i2c/i2c.h>
#include <bsp/bsp.h>

#include <thread>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

/** \cond */
#define NAME_MAX_LENGTH 64
#define LNS_ANGLE 60
#define LNS_LAT 600025472
#define LNS_LNG 278573184

#if ALT_SRC == 1
    std::thread barometerThread;
#endif

#if COORD_SRC == 1
    char bspUart[] = "uart3";
    char gpsUart[] = "serial@7e201600";
#elif COORD_SRC == 2
    #define LATLON_TO_M 0.011131884502145034
    char bspUart[] = "uart5";
    char gpsUart[] = "serial@7e201a00";
    float lnsSin, lnsCos, lnsScale;
    int32_t prevX, prevY;
#endif
char gpsConfigSuffix[] = "default";
UartHandle gpsUartHandler = NULL;

char barometerI2C[] = "i2c1";
char barometerConfigSuffix[] = "p2-3";
I2cHandle barometerHandler = NULL;

int32_t tempCoefs[3];
int32_t pressCoefs[9];

float temperatureFine;
/** \endcond */

/**
 * \~English Writes a byte of information to I2C interface register.
 * \param[in] reg Register to write to.
 * \param[in] val Value to write.
 * \return Returns 1 on successful write, 0 otherwise.
 * \~Russian Выполняет запись байта информации в регистр интерфейса I2C.
 * \param[in] reg Регистр, куда выполняется запись.
 * \param[in] val Записываемое значение.
 * \return Возвращает 1 при успешной записи, иначе -- 0.
 */
int writeRegister(uint8_t reg, uint8_t val) {
    I2cMsg messages[1];
    uint8_t buf[2] = { reg, val };

    messages[0].addr = 0x76;
    messages[0].flags = 0;
    messages[0].buf = buf;
    messages[0].len = 2;

    Retcode rc = I2cXfer(barometerHandler, 400000, messages, 1);
    if (rc != rcOk)
        return 0;

    return 1;
}

/**
 * \~English Reads 2 bytes of information from I2C interface register.
 * \param[in] reg Register to read from.
 * \param[out] val Read values.
 * \return Returns 1 on successful read, 0 otherwise.
 * \~Russian Выполняет чтение 2 байт информации из регистра интерфейса I2C.
 * \param[in] reg Регистр, откуда выполняется чтение.
 * \param[out] val Прочитанные значение.
 * \return Возвращает 1 при успешном чтении, иначе -- 0.
 */
int readRegister16(uint8_t reg, uint8_t* val) {
    I2cMsg messages[2];
    uint8_t writeBuffer[1] = { reg };
    uint8_t readBuffer[2];

    messages[0].addr = 0x76;
    messages[0].flags = 0;
    messages[0].buf = writeBuffer;
    messages[0].len = 1;

    messages[1].addr = 0x76;
    messages[1].flags = I2C_FLAG_RD;
    messages[1].buf = readBuffer;
    messages[1].len = 2;

    I2cError rc = I2cXfer(barometerHandler, 400000, messages, 2);
    if (rc != rcOk)
        return 0;

    val[0] = readBuffer[0];
    val[1] = readBuffer[1];

    return 1;
}

/**
 * \~English Reads 3 bytes of information from I2C interface register.
 * \param[in] reg Register to read from.
 * \param[out] val Read value.
 * \return Returns 1 on successful read, 0 otherwise.
 * \note Due to the specifics of read bytes combining into a 4-byte value, the method is designed
 * to read temperature and pressure values.
 * \~Russian Выполняет чтение 3 байт информации из регистра интерфейса I2C.
 * \param[in] reg Регистр, откуда выполняется чтение.
 * \param[out] val Прочитанное значение.
 * \return Возвращает 1 при успешном чтении, иначе -- 0.
 * \note Из-за специфики объединения прочитанных байтов информации в 4-байтное значение,
 * метод рассчитан на чтение значений температуры и давления.
 */
int readRegister24(uint8_t reg, int32_t &val) {
    I2cMsg messages[2];
    uint8_t writeBuffer[1] = { reg };
    uint8_t readBuffer[3];

    messages[0].addr = 0x76;
    messages[0].flags = 0;
    messages[0].buf = writeBuffer;
    messages[0].len = 1;

    messages[1].addr = 0x76;
    messages[1].flags = I2C_FLAG_RD;
    messages[1].buf = readBuffer;
    messages[1].len = 3;

    I2cError rc = I2cXfer(barometerHandler, 400000, messages, 2);
    if (rc != rcOk)
        return 0;

    val = ((int32_t)(readBuffer[0]) << 12) | ((int32_t)(readBuffer[1]) << 4) | ((int32_t)(readBuffer[2]) >> 4);
    return 1;
}

/**
 * \~English Reads current temperature from the barometer via I2C interface.
 * \param[out] temperature Current temperature.
 * \return Returns 1 on successful read, 0 otherwise.
 * \~Russian Выполняет чтение значения текущей температуры из барометра через интерфейс I2C.
 * \param[out] temperature Текущая температура.
 * \return Возвращает 1 при успешном чтении, иначе -- 0.
 */
int getTemperature(float &temperature) {
    int32_t temp;
    if (!readRegister24(0xFA, temp))
        return 0;

    double tempFst = (temp / 16384.0 - tempCoefs[0] / 1024.0) * tempCoefs[1];
    double tempSnd = pow(temp / 131072.0 - tempCoefs[0] / 8192.0, 2) * tempCoefs[2];

    temperatureFine = tempFst + tempSnd;
    temperature = temperatureFine / 5120.0;

    return 1;
}

/**
 * \~English Reads current pressure from the barometer via I2C interface.
 * \param[out] pressure Current pressure.
 * \return Returns 1 on successful read, 0 otherwise.
 * \~Russian Выполняет чтение значения текущего давления из барометра через интерфейс I2C.
 * \param[out] pressure Текущее давление.
 * \return Возвращает 1 при успешном чтении, иначе -- 0.
 */
int getPressure(float &pressure) {
    int32_t press;
    if (!readRegister24(0xF7, press))
        return 0;

    double pressFst = (temperatureFine / 2.0) - 64000.0;
    double pressSnd = pow(pressFst, 2) * pressCoefs[5] / 32768.0;
    pressSnd += pressFst * pressCoefs[4] * 2.0;
    pressSnd = (pressSnd / 4.0) + (pressCoefs[3] * 65536.0);
    pressFst = (pressCoefs[2] * pow(pressFst, 2) / 524288.0 + pressFst * pressCoefs[1]) / 524288.0;
    pressFst = (1.0 + pressFst / 32768.0) * pressCoefs[0];

    double pressTrd = 1048576.0 - press;
    pressTrd = (pressTrd - (pressSnd / 4096.0)) * 6250.0 / pressFst;
    pressFst = pressCoefs[8] * pow(pressTrd, 2) / 2147483648.0;
    pressSnd = pressTrd * pressCoefs[7] / 32768.0;

    pressure = pressTrd + (pressFst + pressSnd + pressCoefs[6]) / 16.0;

    return 1;
}

/**
 * \~English Procedure that receives drone altitude from a barometer and updates the current location
 * with the received altitude using \ref setAltitude. It is assumed that this procedure is looped
 * and is performed in a parallel thread.
 * \~Russian Процедура, выполняющая получение данных о высоте дрона от барометра, и обновление текущего местоположения
 * полученной высотой при помощи \ref setAltitude. Предполагается, что данная процедура выполняется циклически в параллельной нити.
 */
void getBarometer() {
    while (true) {
        float temp, press;
        getTemperature(temp);
        getPressure(press);
        float alt = 44330.0 * (1.0 - pow(press / 101325.0, 0.1903));
        int32_t altitude = 100 * alt;
        setAltitude(altitude);
        usleep(500000);
    }
}

void getSensors() {
    bool read;
    uint8_t value;
    int mode, messageType, idx, latSign, lngSign;
    int32_t latitude, longitude;
    char head[8], satsStr[8], dopStr[8], latStr[16], lngStr[16], speedStr[16], xStr[16], yStr[16], zStr[16];

    while (true) {
        read = true;
        messageType = 0;
        mode = 0;
        idx = 0;

        while (read) {
            Retcode rc = UartReadByte(gpsUartHandler, &value);
            if (rc != rcOk)
                continue;
            switch (mode) {
            case 0:
                if (value == '$')
                    mode = 1;
                break;
            case 1: //Header
                if (idx >= 8) {
                    read = false;
                    messageType = 0;
                }
                else if (value == ',') {
                    head[idx] = '\0';
                    if ((head[2] == 'G') && (head[3] == 'G') && (head[4] == 'A')) {
                        mode = 2;
                        messageType = 1;
                    }
                    else if ((head[2] == 'V') && (head[3] == 'T') && (head[4] == 'G')) {
                        mode = 10;
                        messageType = 2;
                    }
                    else if ((head[2] == 'L') && (head[3] == 'N') && (head[4] == 'S')) {
                        mode = 17;
                        messageType = 3;
                    }
                    else
                        mode = 0;
                    idx = 0;
                }
                else {
                    head[idx] = value;
                    idx++;
                }
                break;
            case 2: //UTC time
            case 7: //Quality
            case 10: //True heading
            case 11: //True heading consistency
            case 12: //Magnetic heading
            case 13: //Magnetic heading consistency
            case 14: //Speed 1
            case 15: //Speed 1 units (knots)
                if (value == ',')
                    mode++;
                break;
            case 3: //Lat
                if (idx >= 16) {
                    read = false;
                    messageType = 0;;
                }
                else if (value == ',') {
                    latStr[idx] = '\0';
                    idx = 0;
                    mode = 4;
                }
                else {
                    latStr[idx] = value;
                    idx++;
                }
                break;
            case 4: //Lat dir
                if (value == 'N')
                    latSign = 1;
                else if (value == 'S')
                    latSign = -1;
                else if (value == ',')
                    mode = 5;
                else {
                    read = false;
                    messageType = 0;;
                }
                break;
            case 5: //Lng
                if (idx >= 16) {
                    read = false;
                    messageType = 0;;
                }
                else if (value == ',') {
                    lngStr[idx] = '\0';
                    idx = 0;
                    mode = 6;
                }
                else {
                    lngStr[idx] = value;
                    idx++;
                }
                break;
            case 6: //Lng dir
                if (value == 'E')
                    lngSign = 1;
                else if (value == 'W')
                    lngSign = -1;
                else if (value == ',')
                    mode = 7;
                else {
                    read = false;
                    messageType = 0;;
                }
                break;
            case 8: //Sats
                if (idx >= 8) {
                    read = false;
                    messageType = 0;;
                }
                else if (value == ',') {
                    satsStr[idx] = '\0';
                    idx = 0;
                    mode = 9;
                }
                else {
                    satsStr[idx] = value;
                    idx++;
                }
                break;
            case 9: //Hdop
                if (idx >= 8) {
                    read = false;
                    messageType = 0;;
                }
                else if (value == ',') {
                    dopStr[idx] = '\0';
                    idx = 0;
                    read = false;
                }
                else {
                    dopStr[idx] = value;
                    idx++;
                }
                break;
            case 16: //Speed 2 (km/h)
                if (idx >= 16) {
                    read = false;
                    messageType = 0;
                }
                else if (value == ',') {
                    speedStr[idx] = '\0';
                    idx = 0;
                    read = false;
                }
                else {
                    speedStr[idx] = value;
                    idx++;
                }
                break;
            case 17: // X local coordinate
                if (idx >= 16) {
                    read = false;
                    messageType = 0;;
                }
                else if (value == ',') {
                    xStr[idx] = '\0';
                    idx = 0;
                    mode = 18;
                }
                else {
                    xStr[idx] = value;
                    idx++;
                }
                break;
            case 18: // Y local coordinate
                if (idx >= 16) {
                    read = false;
                    messageType = 0;
                }
                else if (value == ',') {
                    yStr[idx] = '\0';
                    idx = 0;
                    mode = 19;
                }
                else {
                    yStr[idx] = value;
                    idx++;
                }
                break;
            case 19: // Z local coordinate
                if (idx >= 16) {
                    read = false;
                    messageType = 0;
                }
                else if (value == '*') {
                    zStr[idx] = '\0';
                    idx = 0;
                    read = false;
                }
                else {
                    zStr[idx] = value;
                    idx++;
                }
                break;
            }
        }

        if (messageType == 1) {
#if COORD_SRC == 1
            longitude = round(10000000 * atof(lngStr + 3) / 60.0f);
            latitude = round(10000000 * atof(latStr + 2) / 60.0f);
            lngStr[3] = '\0';
            latStr[2] = '\0';
            longitude += 10000000 * atoi(lngStr);
            latitude += 10000000 * atoi(latStr);

            setCoords(latitude, longitude);
            setInfo(atof(dopStr), atoi(satsStr));
#endif
        }
        else if (messageType == 2) {
#if COORD_SRC == 1
            setSpeed(atof(speedStr) / 3.6f);
#endif
        }
        else if (messageType == 3) {
#if COORD_SRC == 2
            float lns_x = atof(xStr) / 100.0f;
            float lns_y = atof(yStr) / 100.0f;
            float X = lnsCos * lns_x - lnsSin * lns_y;
            float Y = lnsSin * lns_x + lnsCos * lns_y;
            int32_t difLat = (int32_t)round(Y / LATLON_TO_M);
            int32_t difLng = (int32_t)round((X / LATLON_TO_M) / lnsScale);

            setCoords(LNS_LAT + difLat, LNS_LNG + difLng);
            setInfo(1.0, 4);
#endif
#if ALT_SRC == 2
            setAltitude((int32_t)round(atof(zStr)));
#endif
        }
        else
            logEntry("Failed to parse NMEA string from GPS", ENTITY_NAME, LogLevel::LOG_WARNING);
    }
}

int initNavigationSystem() {
    while (!waitForInit("periphery_controller_connection", "PeripheryController")) {
        logEntry("Failed to receive initialization notification from Periphery Controller. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    char boardName[NAME_MAX_LENGTH] = {0};
    if (KnHalGetEnv("board", boardName, sizeof(boardName)) != rcOk) {
        logEntry("Failed to get board name", ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }

    char gpsConfig[NAME_MAX_LENGTH];
    if (snprintf(gpsConfig, NAME_MAX_LENGTH, "%s.%s", boardName, gpsConfigSuffix) < 0) {
        logEntry("Failed to generate UART config name", ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }
    char barometerConfig[NAME_MAX_LENGTH];
    if (snprintf(barometerConfig, NAME_MAX_LENGTH, "%s.%s", boardName, barometerConfigSuffix) < 0) {
        logEntry("Failed to generate I2C config name", ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }

    char logBuffer[256] = {0};
    Retcode rc = BspInit(NULL);
    if (rc != BSP_EOK) {
        snprintf(logBuffer, 256, "Failed to initialize BSP (" RETCODE_HR_FMT ")", RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return EXIT_FAILURE;
    }
    rc = BspEnableModule(bspUart);
    if (rc != BSP_EOK) {
        snprintf(logBuffer, 256, "Failed to enable UART %s (" RETCODE_HR_FMT ")", RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return EXIT_FAILURE;
    }
    rc = BspSetConfig(bspUart, gpsConfig);
    if (rc != BSP_EOK) {
        snprintf(logBuffer, 256, "Failed to set BSP config for UART %s (" RETCODE_HR_FMT ")", gpsUart, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }
    rc = UartInit();
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to initialize UART (" RETCODE_HR_FMT ")", RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }

    rc = BspEnableModule(barometerI2C);
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to enable I2C %s (" RETCODE_HR_FMT ")", barometerI2C, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }
    rc = BspSetConfig(barometerI2C, barometerConfig);
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to set BSP config for I2C %s (" RETCODE_HR_FMT ")", barometerI2C, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }
    rc = I2cInit();
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to initialize I2C (" RETCODE_HR_FMT ")", RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }

    return 1;
}

int initSensors() {
    char logBuffer[256] = {0};
    Retcode rc = UartOpenPort(gpsUart, &gpsUartHandler);
    if (rc != UART_EOK) {
        snprintf(logBuffer, 256, "Failed to open UART %s (" RETCODE_HR_FMT ")", gpsUart, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

#if COORD_SRC == 1
    rtl_size_t writtenBytes;
    uint8_t gnssNmea[] = { 0xb5, 0x62,
        0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xd0, 0x08, 0x00, 0x00, 0x00, 0xc2, 0x01, 0x00,
        0x03, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbb, 0x58 };
    ssize_t expectedSize = sizeof(gnssNmea);
    rc = UartWrite(gpsUartHandler, gnssNmea, expectedSize, NULL, &writtenBytes);
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to write configuration message to GPS (" RETCODE_HR_FMT ")", RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
    else if (writtenBytes != expectedSize) {
        snprintf(logBuffer, 256, "Failed to write configuration message to GPS: %ld bytes were expected, %ld bytes were sent", expectedSize, writtenBytes);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
    uint8_t gnssSystems[] =  { 0xb5, 0x62,
        0x06, 0x8a, 0x4a, 0x00, 0x00, 0x07, 0x00, 0x00, 0x1f, 0x00, 0x31, 0x10, 0x01, 0x01, 0x00, 0x31,
        0x10, 0x01, 0x20, 0x00, 0x31, 0x10, 0x00, 0x05, 0x00, 0x31, 0x10, 0x00, 0x21, 0x00, 0x31, 0x10,
        0x01, 0x07, 0x00, 0x31, 0x10, 0x01, 0x22, 0x00, 0x31, 0x10, 0x00, 0x0d, 0x00, 0x31, 0x10, 0x00,
        0x0f, 0x00, 0x31, 0x10, 0x00, 0x24, 0x00, 0x31, 0x10, 0x00, 0x12, 0x00, 0x31, 0x10, 0x00, 0x14,
        0x00, 0x31, 0x10, 0x00, 0x25, 0x00, 0x31, 0x10, 0x01, 0x18, 0x00, 0x31, 0x10, 0x01, 0xa7, 0x51 };
    expectedSize = sizeof(gnssSystems);
    rc = UartWrite(gpsUartHandler, gnssSystems, expectedSize, NULL, &writtenBytes);
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to write configuration message to GPS (" RETCODE_HR_FMT ")", RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
    else if (writtenBytes != expectedSize) {
        snprintf(logBuffer, 256, "Failed to write configuration message to GPS: %ld bytes were expected, %ld bytes were sent", expectedSize, writtenBytes);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
#elif COORD_SRC == 2
    float lnsAngle = (LNS_ANGLE * M_PI / 180.0f);
    lnsSin = sin(lnsAngle);
    lnsCos = cos(lnsAngle);
    lnsScale = cos(LNS_LAT * 1.0e-7 * M_PI / 180.0f);
    if (lnsScale < 0.01f)
        lnsScale = 0.01f;
    prevX = 0;
    prevY = 0;
#endif

#if ALT_SRC == 1
    rc = I2cOpenChannel(barometerI2C, &barometerHandler);
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to open I2C %s (" RETCODE_HR_FMT ")", barometerI2C, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    if (!writeRegister(0xF5, 0x90)) {
        logEntry("Failed to set barometer filter", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
    if (!writeRegister(0xF4, 0x57)) {
        logEntry("Failed to set barometer sampling rates", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    for (int i = 0; i < 3; i++) {
        uint8_t value[2];
        if (!readRegister16(0x88 + i * 2, value)) {
            logEntry("Failed to read barometer temperature coefficient", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        if (i) {
            int16_t signedValue;
            memcpy(&signedValue, &value, sizeof(int16_t));
            tempCoefs[i] = signedValue;
        }
        else {
            uint16_t unsignedValue;
            memcpy(&unsignedValue, &value, sizeof(uint16_t));
            tempCoefs[i] = unsignedValue;
        }
    }

    for (int i = 0; i < 9; i++) {
        uint8_t value[2];
        if (!readRegister16(0x8E + i * 2, value)) {
            logEntry("Failed to read barometer pressure coefficient", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        if (i) {
            int16_t signedValue;
            memcpy(&signedValue, &value, sizeof(int16_t));
            pressCoefs[i] = signedValue;
        }
        else {
            uint16_t unsignedValue;
            memcpy(&unsignedValue, &value, sizeof(uint16_t));
            pressCoefs[i] = unsignedValue;
        }
    }

    barometerThread = std::thread(getBarometer);
#endif

    return 1;
}