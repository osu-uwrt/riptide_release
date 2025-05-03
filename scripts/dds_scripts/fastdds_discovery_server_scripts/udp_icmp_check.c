/*
 * A dumb sprogram to attempt to ping the discovery server over UDP and wait for
 * an ICMP error response to check the discovery server is listening on the port.
 *
 * This file is compiled and managed by check_discovery_server_alive.bash, do not use
 * directly (see below for why).
 *
 * This sends an empty UDP packet to the [host] on the specified [port]
 * This waits for [timeout milliseconds] for an unreachable port ICMP error
 * If no ICMP error response is received within timeout, we'll assume the discovery
 * server is online. This is unfortunately not foolproof (if the host doesn't send
 * icmp error responses in time) and slow (since it needs to wait for the timeout
 * to expire) but the RTPS protocol is dumb and doesn't have a simple way to ping.
 *
 * This script isn't that useful on its own, since it'll say everything is fine if
 * no repsonse is received. However, (assuming a firewall isn't enabled), after
 * the ping command reports a success, it can be safely assumed that if we don't
 * receive an ICMP error packet then the host is probably listening on the port.
 *
 * Again, not ideal, but also, RTPS is dumb and won't give a straightforward way
 * to check connection (and I'm not about to try to build a whole RTPS participant
 * just so I can see if the server is online, and do it in a way that won't break
 * the discovery server in wacky ways).
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/poll.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#define BUF_SIZE 500

int main(int argc, char *argv[]) {
    struct addrinfo hints;
    struct addrinfo *result, *rp;
    int sfd, s, j;
    ssize_t nread;
    char buf[BUF_SIZE];

    if (argc != 4) {
        fprintf(stderr, "Usage: %s [host] [port] [timeout milliseconds]\n", argv[0]);
        return 2;
    }

    /* Obtain address(es) matching host/port */

    memset(&hints, 0, sizeof(struct addrinfo));
    hints.ai_family = AF_UNSPEC;    /* Allow IPv4 or IPv6 */
    hints.ai_socktype = SOCK_DGRAM; /* Datagram socket */
    hints.ai_flags = 0;
    hints.ai_protocol = 0;          /* Any protocol */

    int timeout = atoi(argv[3]);
    if (timeout == 0) {
        fprintf(stderr, "Invalid timeout\n");
        return 2;
    }

    s = getaddrinfo(argv[1], argv[2], &hints, &result);
    if (s != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(s));
        return 2;
    }

    /* getaddrinfo() returns a list of address structures.
    Try each address until we successfully connect(2).
    If socket(2) (or connect(2)) fails, we (close the socket
    and) try the next address. */

    for (rp = result; rp != NULL; rp = rp->ai_next) {
        sfd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (sfd == -1)
            continue;

        int recverr_en = 1;
        if (setsockopt(sfd, SOL_IP, IP_RECVERR, &recverr_en, sizeof(recverr_en))) {
            perror("setsockopt(SOL_IP, IP_RECVERR, 1)");
            return 2;
        }

        char data = 0;
        if (sendto(sfd, &data, sizeof(data), 0, rp->ai_addr, rp->ai_addrlen) == sizeof(data)) {
            break;
        }

        close(sfd);
    }

    if (rp == NULL) {               /* No address succeeded */
        perror("sendto");
        return 2;
    }

    freeaddrinfo(result);           /* No longer needed */

    struct pollfd fd = {.events = POLLERR, .fd = sfd};

    int rc = poll(&fd, 1, 100);
    if (rc < 0) {
        perror("poll");
    }
    else if (rc > 0) {
        // We have data in the poll
        // That means that there must be error data present in the socket (ICMP errors)
        // Report error status code
        // Bash will handle the ways to print this nicely
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
