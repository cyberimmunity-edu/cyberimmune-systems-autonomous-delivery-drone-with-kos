#pragma once

int initPeripheryController();
int initGpioPins();

int setBuzzer(bool enable);
int setKillSwitch(bool enable);
int setCargoLock(bool enable);