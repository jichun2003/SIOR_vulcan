#include "json_out.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

/* macOS 등에는 MSG_NOSIGNAL이 없다 (리눅스 전용). 어차피 main에서 SIGPIPE를 무시하므로
 * 없는 환경에서는 0으로 두면 동작이 같다. */
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

static void set_nonblock(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
}

int aig_json_open(aig_json_server_t *s, int port)
{
    struct sockaddr_in addr;
    int yes = 1;

    memset(s, 0, sizeof(*s));
    for (int i = 0; i < AIG_JSON_MAX_CLIENTS; i++) {
        s->clients[i] = -1;
    }

    s->listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (s->listen_fd < 0) {
        fprintf(stderr, "[json] socket 실패: %s\n", strerror(errno));
        return -1;
    }
    setsockopt(s->listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    set_nonblock(s->listen_fd);

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)port);
    if (bind(s->listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        fprintf(stderr, "[json] bind(%d) 실패: %s\n", port, strerror(errno));
        close(s->listen_fd);
        s->listen_fd = -1;
        return -1;
    }
    if (listen(s->listen_fd, AIG_JSON_MAX_CLIENTS) != 0) {
        fprintf(stderr, "[json] listen 실패: %s\n", strerror(errno));
        close(s->listen_fd);
        s->listen_fd = -1;
        return -1;
    }
    printf("[json] TCP %d 포트에서 검출 결과를 내보냅니다 (한 줄 = JSON 한 개)\n", port);
    return 0;
}

void aig_json_poll(aig_json_server_t *s)
{
    if (s->listen_fd < 0) {
        return;
    }
    for (;;) {
        struct sockaddr_in peer;
        socklen_t plen = sizeof(peer);
        int fd = accept(s->listen_fd, (struct sockaddr *)&peer, &plen);
        int yes = 1;

        if (fd < 0) {
            return;             /* EAGAIN = 새 접속 없음 */
        }
        if (s->n_clients >= AIG_JSON_MAX_CLIENTS) {
            close(fd);
            continue;
        }
        set_nonblock(fd);
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
        for (int i = 0; i < AIG_JSON_MAX_CLIENTS; i++) {
            if (s->clients[i] < 0) {
                s->clients[i] = fd;
                s->n_clients++;
                break;
            }
        }
        printf("[json] 접속: %s\n", inet_ntoa(peer.sin_addr));
    }
}

void aig_json_broadcast(aig_json_server_t *s, const char *line)
{
    size_t len = strlen(line);

    for (int i = 0; i < AIG_JSON_MAX_CLIENTS; i++) {
        ssize_t n;
        int fd = s->clients[i];

        if (fd < 0) {
            continue;
        }
        n = send(fd, line, len, MSG_NOSIGNAL);
        if (n >= 0) {
            send(fd, "\n", 1, MSG_NOSIGNAL);
            continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            continue;           /* 받는 쪽이 느림 -> 이 줄은 버린다 */
        }
        printf("[json] 접속 끊김 (%s)\n", strerror(errno));
        close(fd);
        s->clients[i] = -1;
        s->n_clients--;
    }
}

void aig_json_close(aig_json_server_t *s)
{
    for (int i = 0; i < AIG_JSON_MAX_CLIENTS; i++) {
        if (s->clients[i] >= 0) {
            close(s->clients[i]);
            s->clients[i] = -1;
        }
    }
    s->n_clients = 0;
    if (s->listen_fd >= 0) {
        close(s->listen_fd);
        s->listen_fd = -1;
    }
}
