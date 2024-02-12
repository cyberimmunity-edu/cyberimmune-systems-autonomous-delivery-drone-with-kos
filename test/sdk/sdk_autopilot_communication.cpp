#include "sdk_autopilot_communication.h"
#include "sdk_firmware.h"
#include "sdk_net.h"

#ifdef FOR_SITL
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>
#else
#include <rtl/retcode_hr.h>
#include <uart/uart.h>
#endif

#include <stdio.h>

static const uint8_t commandMessageHead[KOS_COMMAND_MESSAGE_HEAD_SIZE] = { 0x7a, 0xfe, 0xf0, 0x0d };

#ifdef FOR_SITL
int uartSocket = NULL;
char sitlIp[] = "172.28.65.87";
uint16_t sitlPort = 5765;
#else
UartHandle uartH = NULL;
#endif

KOSCommandMessage::KOSCommandMessage() {
    for (int i = 0; i < KOS_COMMAND_MESSAGE_HEAD_SIZE; i++)
        head[i] = 0;
    command = KOSCommand::ERROR;
}

KOSCommandMessage::KOSCommandMessage(KOSCommand com) {
    memcpy(head, commandMessageHead, KOS_COMMAND_MESSAGE_HEAD_SIZE);
    command = (uint8_t)com;
}

int openUartChannel() {
#ifdef FOR_SITL
    struct sockaddr_in sitlAddress = { 0 };

    uartSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (uartSocket == -1) {
        fprintf(stderr, "Failed to create socket\n");
        return 0;
    }

    sitlAddress.sin_family = AF_INET;
    sitlAddress.sin_addr.s_addr = inet_addr(sitlIp);
    sitlAddress.sin_port = htons(sitlPort);

    if (connect(uartSocket, (struct sockaddr*)&sitlAddress, sizeof(sitlAddress)) != 0) {
        fprintf(stderr, "Failed to connect to %s:%d\n", sitlIp, sitlPort);
        return 0;
    }
#else
    char* channel = getChannelName(firmwareChannel::UART);
    Retcode rc = UartOpenPort(channel, &uartH);
    if (rc != rcOk) {
        fprintf(stderr, "UartOpenPort() %s failed: "RETCODE_HR_FMT"\n", channel, RETCODE_HR_PARAMS(rc));
        return 0;
    }
#endif

    return 1;
}

int initializeUart() {
#ifdef FOR_SITL
    if ((uartSocket == NULL) && !openUartChannel())
        return 0;
#else
    if ((uartH == NULL) && !openUartChannel())
        return 0;
#endif

    return 1;
}

#ifdef FOR_SITL
int initializeSitlUart() {
    return initializeUart();
}
#endif

int readNBytes(uint32_t expectedSize, void* readDestination) {
#ifdef FOR_SITL
            ssize_t readBytes = read(uartSocket, readDestination, expectedSize);
            if (readBytes != expectedSize) {
                fprintf(stderr, "Error reading message from Ardupilot. %d bytes were expected, %d bytes were received\n", expectedSize, readBytes);
                return 0;
            }
#else
            rtl_size_t readBytes;
            Retcode rc = UartRead(uartH, (rtl_uint8_t*)readDestination, expectedSize, NULL, &readBytes);
            if (rc != rcOk) {
                fprintf(stderr, "UartRead() failed: "RETCODE_HR_FMT"\n", RETCODE_HR_PARAMS(rc));
                return 0;
            }
            else if (readBytes != expectedSize) {
                fprintf(stderr, "Error reading message from Ardupilot. %d bytes were expected, %d bytes were received\n", expectedSize, readBytes);
                return 0;
            }
#endif

    return 1;
}

int sendCommand(KOSCommand com) {
    if (!initializeUart())
        return 0;

    KOSCommandMessage command = KOSCommandMessage(com);
    uint8_t message[sizeof(KOSCommandMessage)];
    memcpy(message, &command, sizeof(KOSCommandMessage));

#ifdef FOR_SITL
    write(uartSocket, message, sizeof(KOSCommandMessage));
#else
    for (int i = 0; i < sizeof(KOSCommandMessage); i++) {
        Retcode rc = UartWriteByte(uartH, message[i]);
        if (rc != rcOk) {
            fprintf(stderr, "UartWriteByte is failed: "RETCODE_HR_FMT"\n", RETCODE_HR_PARAMS(rc));
            return 0;
        }
    }
#endif

    return 1;
}

int waitForCommand() {
    if (!initializeUart())
        return 0;

    while (1) {
        uint8_t message[sizeof(KOSCommand)];
        bool restart = false;
        for (int i = 0; i < KOS_COMMAND_MESSAGE_HEAD_SIZE; i++) {

#ifdef FOR_SITL
            ssize_t readBytes = read(uartSocket, message + i, 1);
            if (readBytes != 1) {
                fprintf(stderr, "Failed to read\n");
                return 0;
            }
#else
            Retcode rc = UartReadByte(uartH, message + i);
            if (rc != rcOk) {
                fprintf(stderr, "UartReadByte() failed: "RETCODE_HR_FMT"\n", RETCODE_HR_PARAMS(rc));
                return 0;
            }
#endif

            if (message[i] != commandMessageHead[i]) {
                fprintf(stderr, "KOS Message Error: Unknown message head %d at place %d\n", message[i], i);
                restart = true;
                break;
            }
        }

        if (!restart) {
            if (!readNBytes(sizeof(KOSCommandMessage) - KOS_COMMAND_MESSAGE_HEAD_SIZE, message + KOS_COMMAND_MESSAGE_HEAD_SIZE))
                return 0;

            KOSCommandMessage commandMessage;
            memcpy(&commandMessage, message, sizeof(KOSCommandMessage));
            switch (commandMessage.command) {
                case KOSCommand::ArmRequest:
                    fprintf(stderr, "Arm request is received\n");
                    return 1;
                    break;
                default:
                    fprintf(stderr, "KOS Command Message Error: Unknown command %d is received\n", (int)commandMessage.command);
                    break;
            }
        }
    }
    return 1;
}