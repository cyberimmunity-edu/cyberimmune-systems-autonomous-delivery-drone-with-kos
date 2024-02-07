#include "sdk_uart.h"
#include "sdk_firmware.h"
#include <rtl/retcode_hr.h>
#include <uart/uart.h>
#include <stdio.h>

static const uint8_t commandMessageHead[KOS_COMMAND_MESSAGE_HEAD_SIZE] = { 0x7a, 0xfe, 0xf0, 0x0d };

UartHandle uartH = NULL;

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
    char* channel = getChannelName(firmwareChannel::UART);
    Retcode rc = UartOpenPort(channel, &uartH);
    if (rc != rcOk) {
        fprintf(stderr, "UartOpenPort() %s failed: "RETCODE_HR_FMT"\n", channel, RETCODE_HR_PARAMS(rc));
        return 0;
    }
    return 1;
}

int initializeUart() {
    if ((uartH == NULL) && !openUartChannel())
        return 0;

    return 1;
}

int waitForCommand() {
    if (!initializeUart())
        return 0;

    while (1) {
        uint8_t message[sizeof(KOSCommand)];
        bool restart = false;
        for (int i = 0; i < KOS_COMMAND_MESSAGE_HEAD_SIZE; i++) {
            Retcode rc = UartReadByte(uartH, message + i);
            if (rc != rcOk) {
                fprintf(stderr, "UartReadByte() failed: "RETCODE_HR_FMT"\n", RETCODE_HR_PARAMS(rc));
                return 0;
            }
            if (message[i] != commandMessageHead[i]) {
                fprintf(stderr, "KOS Message Error: Unknown message head %d at place %d\n", message[i], i);
                restart = true;
                break;
            }
        }
        if (!restart) {
            rtl_size_t bytes_read;
            ssize_t expected_size = sizeof(KOSCommandMessage) - KOS_COMMAND_MESSAGE_HEAD_SIZE;
            Retcode rc = UartRead(uartH, message + KOS_COMMAND_MESSAGE_HEAD_SIZE, expected_size, NULL, &bytes_read);
            if (rc != rcOk) {
                fprintf(stderr, "UartRead() failed: "RETCODE_HR_FMT"\n", RETCODE_HR_PARAMS(rc));
                return 0;
            }
            else if (bytes_read != expected_size) {
                fprintf(stderr, "KOS Command Message Error: Message is shorter than expected (%ld vs %ld)\n", bytes_read + KOS_COMMAND_MESSAGE_HEAD_SIZE, sizeof(KOSCommandMessage));
                return 0;
            }
            KOSCommandMessage command_message;
            memcpy(&command_message, message, sizeof(KOSCommandMessage));
            switch (command_message.command) {
                case KOSCommand::ArmRequest:
                    fprintf(stderr, "Arm request is received\n");
                    return 1;
                    break;
                default:
                    fprintf(stderr, "KOS Command Message Error: Unknown command %d is received\n", (int)command_message.command);
                    break;
            }
        }
    }
    return 1;
}

int sendCommand(KOSCommand com) {
    if (!initializeUart())
        return 0;

    KOSCommandMessage command = KOSCommandMessage(com);
    uint8_t message[sizeof(KOSCommandMessage)];
    memcpy(message, &command, sizeof(KOSCommandMessage));
    for (int i = 0; i < sizeof(KOSCommandMessage); i++) {
        Retcode rc = UartWriteByte(uartH, message[i]);
        if (rc != rcOk) {
            fprintf(stderr, "UartWriteByte is failed: "RETCODE_HR_FMT"\n", RETCODE_HR_PARAMS(rc));
            return 0;
        }
    }
    return 1;
}