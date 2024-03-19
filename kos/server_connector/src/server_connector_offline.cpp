#include "../include/server_connector.h"

#include <string.h>

int initServerConnector() {
    return 1;
}

int sendRequest(char* query, char* response) {
    if (strstr(query, "auth?") != NULL)
        strcpy(response, "$Success#");
    else if (strstr(query, "fmission_kos?") != NULL)
        strcpy(response, "$FlightMission H-35.3633463_149.1652273_587.05&T5.0&W0.0_-35.3631122_149.1651964_5.0&W0.0_-35.3632741_149.1653869_2.0&W0.0_-35.3630641_149.1655062_5.0&W0.0_-35.3633091_149.1655652_5.0&S10.0_2000.0&W0.0_-35.3634010_149.1654137_5.0&W0.0_-35.3596913_149.1652273_5.0&L-35.3633463_149.1652273_587.05#");
    else if ((strstr(query, "arm?") != NULL) || (strstr(query, "fly_accept?") != NULL))
        strcpy(response, "$Arm: 0#");
    else
        strcpy(response, "$#");

    return 1;
}