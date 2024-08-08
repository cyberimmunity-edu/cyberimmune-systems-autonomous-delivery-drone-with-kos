#include "../include/server_connector.h"

#include <stdio.h>
#include <string.h>

int initServerConnector() {
    return 1;
}

int sendRequest(char* query, char* response) {
    if (strstr(query, "/api/kill_switch?") != NULL)
        strcpy(response, "$KillSwitch: 1#");
    else if (strstr(query, "/api/auth?") != NULL)
        strcpy(response, "$Success#");
    else if (strstr(query, "/api/fmission_kos?") != NULL)
        strcpy(response, "$FlightMission H53.1019446_107.3774394_846.22&T5.0&W0.0_53.1020863_107.3774180_5.0&W0.0_53.1021926_107.3775065_5.0&W0.0_53.1023102_107.3776701_5.0&W0.0_53.1023682_107.3779464_5.0&W0.0_53.1023923_107.3782736_5.0&W0.0_53.1023279_107.3786089_5.0&W0.0_53.1021991_107.3787698_5.0&S5.0_1200.0&W0.0_53.1020284_107.3788181_5.0&W0.0_53.1018818_107.3786679_5.0&W0.0_53.1018206_107.3782790_5.0&W0.0_53.1017900_107.3778149_5.0&W0.0_53.1018480_107.3775575_5.0&W0.0_53.1019446_107.3774394_5.0&L53.1019446_107.3774394_846.22#");
    else if (strstr(query, "/api/get_all_forbidden_zones?") != NULL)
        strcpy(response, "$ForbiddenZones 1&7&53.1021169_107.3777130&53.1022184_107.3779973&53.1022023_107.3783299&53.1020767_107.3784882&53.1019962_107.3782709&53.1019189_107.3779812&53.1019656_107.3777157#");
    else if ((strstr(query, "/api/arm?") != NULL) || (strstr(query, "/api/fly_accept?") != NULL))
        strcpy(response, "$Arm: 0#");
    else
        strcpy(response, "$#");

    return 1;
}