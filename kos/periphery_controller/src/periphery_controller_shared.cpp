#include "../include/periphery_controller.h"

#include <thread>

std::thread buzzerThread;
bool buzzerEnabled = false;
int buzzTime = 2;

void buzz() {
    buzzerEnabled = true;
    setBuzzer(true);
    clock_t startTime = clock();
    while (true) {
        clock_t time = clock() - startTime;
        if ((time / CLOCKS_PER_SEC) >= buzzTime)
            break;
    }
    setBuzzer(false);
    buzzerEnabled = false;
}

int enableBuzzer() {
    if (buzzerEnabled)
        return 0;
    if (buzzerThread.joinable())
        buzzerThread.join();
    buzzerThread = std::thread(buzz);
    return 1;
}