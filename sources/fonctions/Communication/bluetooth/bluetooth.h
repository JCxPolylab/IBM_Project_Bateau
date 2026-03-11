#ifndef __BLUETOOTH_API__H__
#define __BLUETOOTH_API__H__
/*
#include <string>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <MAC> <commande>\n"
            << "Ex: " << argv[0] << " AA:BB:CC:DD:EE:FF \"PING\"\n";
        return 1;
    }

    const char* dest = argv[1];
    const char* cmd = argv[2];

    int sock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
    if (sock < 0) { perror("socket"); return 1; }

    sockaddr_rc addr{};
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = 1;
    str2ba(dest, &addr.rc_bdaddr);

    if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("connect");
        close(sock);
        return 1;
    }

    std::string msg = std::string(cmd) + "\n";
    write(sock, msg.c_str(), msg.size());

    char buf[1024] = { 0 };
    int bytes = read(sock, buf, sizeof(buf) - 1);
    if (bytes > 0) std::cout << "Réponse: " << buf;

    close(sock);
    return 0;
}
*/

#endif // !__BLUETOOTH_API__H__
