#include <stdlib.h>

char* getMockLog();
bool getMockBuzzer();
void setMockBuzzer(bool enable);

//credential_manager_online.cpp
uint8_t hexCharToInt(char c);
void stringToBytes(char* source, uint32_t sourceSize, uint8_t* destination);

//credential_manager_shared.cpp
void hashToKey(uint8_t* source, uint32_t sourceSize, uint8_t* destination);
void bytesToString(uint8_t* source, char* destination);

//mission.cpp
int isStopSymbol(char character);
int parseInt(char*& string, int32_t& value, uint32_t numAfterPoint);
int parseCommands(char* str);

//navigation_system_shared.cpp
bool hasPosition();
void setGpsInfo(float dop, int32_t sats);
int getGpsInfo(float& dop, int32_t& sats);
void setAltitude(int32_t altitude);
void setCoords(int32_t latitude, int32_t longitude);
int getCoords(int32_t &latitude, int32_t &longitude, int32_t &altitude);

//periphery_controller_shared.cpp
void buzz();
int enableBuzzer();