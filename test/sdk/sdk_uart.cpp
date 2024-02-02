#include "sdk_uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <bsp/bsp.h>
#include <rtl/retcode_hr.h>

#define UART_NAME "uart3"
#define UART_CONFIG "rpi4_bcm2711.default"

static const uint8_t command_message_head[KOS_COMMAND_MESSAGE_HEAD_SIZE] = { 0x7a, 0xfe, 0xf0, 0x0d };

UartHandle uartH = KDF_INVALID_HANDLE;

KOSCommandMessage::KOSCommandMessage() {
    for (int i = 0; i < KOS_COMMAND_MESSAGE_HEAD_SIZE; i++)
        head[i] = 0;
    command = KOSCommandType::ERROR;
}

KOSCommandMessage::KOSCommandMessage(KOSCommandType com) {
    memcpy(head, command_message_head, KOS_COMMAND_MESSAGE_HEAD_SIZE);
    command = (uint8_t)com;
}

int startUART(void) {
    Retcode rc = BspEnableModule(UART_NAME);
    if (rc != BSP_EOK) {
        fprintf(stderr, "BspEnableModule() failed: "RETCODE_HR_FMT"\n", RETCODE_HR_PARAMS(rc));
        return EXIT_FAILURE;
    }
    rc = BspSetConfig(UART_NAME, UART_CONFIG);
    if (rc != BSP_EOK) {
        fprintf(stderr, "BspSetConfig() failed: "RETCODE_HR_FMT"\n", RETCODE_HR_PARAMS(rc));
        return EXIT_FAILURE;
    }
    rc = UartInit();
    if (rc != UART_EOK) {
        fprintf(stderr, "UartInit() failed: "RETCODE_HR_FMT"\n", RETCODE_HR_PARAMS(rc));
        return EXIT_FAILURE;
    }
    rc = UartOpenPort(UART_NAME, &uartH);
    if (rc != UART_EOK) {
        fprintf(stderr, "UartOpenPort() %s failed: "RETCODE_HR_FMT"\n", UART_NAME, RETCODE_HR_PARAMS(rc));
        return 0;
    }
    return 1;
}

int wait_for_ardupilot_request() {
    fprintf(stderr, "DEBUG: Start listening\n");
    while (1) {
        fprintf(stderr, "DEBUG: Start again\n");
        uint8_t message[sizeof(KOSCommandMessage)];
        bool restart = false;
        for (int i = 0; i < KOS_COMMAND_MESSAGE_HEAD_SIZE; i++) {
            Retcode rc = UartReadByte(uartH, message + i);
            if (rc != UART_EOK) {
                fprintf(stderr, "UartReadByte() %s failed: "RETCODE_HR_FMT"\n", UART_NAME, RETCODE_HR_PARAMS(rc));
                return 0;
            }
            if (message[i] != command_message_head[i]) {
                fprintf(stderr, "KOS Message Error: Unknown message head %d at place %d\n", message[i], i);
                restart = true;
                break;
            }
            fprintf(stderr, "DEBUG: Received byte %d\n", message[i]);
        }
        if (!restart) {
            rtl_size_t bytes_read;
            ssize_t expected_size = sizeof(KOSCommandMessage) - KOS_COMMAND_MESSAGE_HEAD_SIZE;
            Retcode rc = UartRead(uartH, message + KOS_COMMAND_MESSAGE_HEAD_SIZE, expected_size, NULL, &bytes_read);
            if (rc != UART_EOK) {
                fprintf(stderr, "UartRead() %s failed: "RETCODE_HR_FMT"\n", UART_NAME, RETCODE_HR_PARAMS(rc));
                return 0;
            }
            else if (bytes_read != expected_size) {
                fprintf(stderr, "KOS Command Message Error: Message is shorter than expected (%ld vs %ld)\n", bytes_read + KOS_COMMAND_MESSAGE_HEAD_SIZE, sizeof(KOSCommandMessage));
                return 0;
            }
            for (int i = 0; i < expected_size; i++)
                fprintf(stderr, "DEBUG: Byte %d\n", message[KOS_COMMAND_MESSAGE_HEAD_SIZE + i]);
            fprintf(stderr, "DEBUG: Full message received\n");
            KOSCommandMessage command_message;
            memcpy(&command_message, message, sizeof(KOSCommandMessage));
            switch (command_message.command) {
                case KOSCommandType::Command_ArmRequest:
                    fprintf(stderr, "Arm request is received\n");
                    return 1;
                    break;
                case KOSCommandType::Command_ArmForbid:
                    fprintf(stderr, "KOS Command Message Error: Unknown command %d is received\n", (int)command_message.command);
                    break;
            }
        }
    }
    fprintf(stderr, "DEBUG: Finish listening\n");
    return 1;
}

int send_command(KOSCommandType com) {
    KOSCommandMessage command = KOSCommandMessage(com);
    uint8_t message[sizeof(KOSCommandMessage)];
    memcpy(message, &command, sizeof(KOSCommandMessage));
    for (int i = 0; i < sizeof(KOSCommandMessage); i++) {
        Retcode rc = UartWriteByte(uartH, message[i]);
        if (rc != UART_EOK) {
            fprintf(stderr, "UartWriteByte is failed: "RETCODE_HR_FMT"\n", RETCODE_HR_PARAMS(rc));
            return 0;
        }
    }
    return 1;
}