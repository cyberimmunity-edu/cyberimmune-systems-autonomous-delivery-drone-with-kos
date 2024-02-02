#pragma once

#include <string>
#include <kos_net.h>

int get_request(std::string address, uint port, std::string query, std::string& response);