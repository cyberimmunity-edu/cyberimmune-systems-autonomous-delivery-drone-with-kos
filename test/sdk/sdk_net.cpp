#include "sdk_net.h"

int get_request(std::string address, uint port, std::string query, std::string& response)
{
    int socket_desc = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socket_desc < 0)
    {
        fprintf(stderr, "Failed to create a socket: %s\n", strerror(errno));
        return 0;
    }

    sockaddr_in serv_addr = {0};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    serv_addr.sin_addr.s_addr = inet_addr(address.c_str());
    if (connect(socket_desc, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
	    fprintf(stderr, "Failed to connect: %s\n", strerror(errno));
		close(socket_desc);
        return 0;
	}

    std::string get_req = "GET /" + query + " HTTP/1.1\r\nHost: " + address + "\r\nConnection: close\r\n\r\n";
    if (send(socket_desc, get_req.c_str(), get_req.size(), 0) < 0) {
		fprintf(stderr, "Failed to send a request: %s\n", strerror(errno));
	    close(socket_desc);
		return 0;
	}

    ssize_t resp_len;
    char buffer[1024];
	response = "";
    while ((resp_len = recv(socket_desc, buffer, sizeof(buffer), 0)) > 0)
		response.append(buffer, resp_len);
    close(socket_desc);
    return 1;
}