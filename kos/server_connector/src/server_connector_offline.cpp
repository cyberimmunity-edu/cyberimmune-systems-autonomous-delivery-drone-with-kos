#include "../include/server_connector.h"

#include <string.h>

int initServerConnector() {
    return 1;
}

int sendRequest(char* query, char* response) {
    if (strstr(query, "auth?") != NULL)
        strcpy(response, "$Success#");
    else if (strstr(query, "fmission_kos?") != NULL)
        strcpy(response, "$FlightMission H-35.3632621_149.1652374_584.09&T120.0&W15.0_-35.3610736_149.1620636_120.0&W0.0_-35.3671981_149.1633296_110.0&S1.0_50.0&W0.0_-35.3643809_149.1589522_110.0&W0.0_-35.3654483_149.1684794_100.0&W0.0_-35.3608811_149.169724_100.0&W0.0_-35.3596912_149.1653681_100.0&L-35.3632621_149.1652374_584.09#");
    else if ((strstr(query, "arm?") != NULL) || (strstr(query, "fly_accept?") != NULL))
        strcpy(response, "$Arm: 0#");
    else
        strcpy(response, "$#");

    return 1;
}