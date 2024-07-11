#pragma once

#include "../../shared/include/ipc_messages_logger.h"

int initPeripheryController();
int initGpioPins();

bool isKillSwitchEnabled();
int setBuzzer(bool enable);

int enableBuzzer();
int setKillSwitch(bool enable);
int setCargoLock(bool enable);

void checkKillSwitchPermission();