#pragma once

int initPeripheryController();
int initGpioPins();

bool isKillSwitchEnabled();
int setBuzzer(bool enable);

int enableBuzzer();
int setKillSwitch(bool enable);
int setCargoLock(bool enable);

void checkKillSwitchPermission();