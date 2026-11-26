#include "preview_out.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#ifdef AIG_HAVE_JPEG
#include <jpeglib.h>
#endif

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

#define HEADER_LEN 16u

static void set_nonblock(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
}

static void put_u32le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xffu);
    p[1] = (uint8_t)((v >> 8) & 0xffu);
    p[2] = (uint8_t)((v >> 16) & 0xffu);
    p[3] = (uint8_t)((v >> 24) & 0xffu);
}

static void drop_client(aig_preview_server_t *s, int i, const char *why)
{
    aig_preview_client_t *c = &s->clients[i];

    printf("[preview] 접속 끊김 (%s)\n", why);
    close(c->fd);
    free(c->buf);
    memset(c, 0, sizeof(*c));
    c->fd = -1;
    s->n_clients--;
}

bool aig_preview_has_jpeg(void)
{
#ifdef AIG_HAVE_JPEG
    return true;
#else
    return false;
#endif
}

int aig_preview_open(aig_preview_server_t *s, int port, int quality)
{
    struct sockaddr_in addr;
    int yes = 1;

    memset(s, 0, sizeof(*s));
    for (int i = 0; i < AIG_PREVIEW_MAX_CLIENTS; i++) {
        s->clients[i].fd = -1;
    }
    s->quality = (quality < 1) ? 1 : (quality > 100 ? 100 : quality);

    s->listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (s->listen_fd < 0) {
        fprintf(stderr, "[preview] socket 실패: %s\n", strerror(errno));
        return -1;
    }
    setsockopt(s->listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    set_nonblock(s->listen_fd);

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)port);
    if (bind(s->listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        fprintf(stderr, "[preview] bind(%d) 실패: %s\n", port, strerror(errno));
        close(s->listen_fd);
        s->listen_fd = -1;
        return -1;
    }
    if (listen(s->listen_fd, AIG_PREVIEW_MAX_CLIENTS) != 0) {
        fprintf(stderr, "[preview] listen 실패: %s\n", strerror(errno));
        close(s->listen_fd);
        s->listen_fd = -1;
        return -1;
    }
    printf("[preview] TCP %d 포트에서 카메라 영상을 내보냅니다 (%s)\n", port,
           aig_preview_has_jpeg() ? "JPEG" : "원시 RGB - libjpeg 없음");
    return 0;
}

/* 한 클라이언트의 밀린 데이터를 보낼 수 있는 만큼 보낸다 */
static void flush_client(aig_preview_server_t *s, int i)
{
    aig_preview_client_t *c = &s->clients[i];

    while (c->off < c->len) {
        ssize_t n = send(c->fd, c->buf + c->off, c->len - c->off, MSG_NOSIGNAL);
        if (n > 0) {
            c->off += (size_t)n;
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            return;             /* 소켓 버퍼가 참 -> 다음 프레임에 이어서 */
        }
        drop_client(s, i, n == 0 ? "closed" : strerror(errno));
        return;
    }
    c->len = 0;
    c->off = 0;
}

void aig_preview_poll(aig_preview_server_t *s)
{
    if (s->listen_fd < 0) {
        return;
    }
    for (;;) {
        struct sockaddr_in peer;
        socklen_t plen = sizeof(peer);
        int fd = accept(s->listen_fd, (struct sockaddr *)&peer, &plen);
        int yes = 1;
        int placed = 0;

        if (fd < 0) {
            break;              /* EAGAIN = 새 접속 없음 */
        }
        set_nonblock(fd);
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
        for (int i = 0; i < AIG_PREVIEW_MAX_CLIENTS; i++) {
            if (s->clients[i].fd < 0) {
                memset(&s->clients[i], 0, sizeof(s->clients[i]));
                s->clients[i].fd = fd;
                s->n_clients++;
                placed = 1;
                break;
            }
        }
        if (!placed) {
            close(fd);
            continue;
        }
        printf("[preview] 접속: %s\n", inet_ntoa(peer.sin_addr));
    }
    for (int i = 0; i < AIG_PREVIEW_MAX_CLIENTS; i++) {
        if (s->clients[i].fd >= 0 && s->clients[i].len > 0) {
            flush_client(s, i);
        }
    }
}

bool aig_preview_wants_frame(const aig_preview_server_t *s)
{
    for (int i = 0; i < AIG_PREVIEW_MAX_CLIENTS; i++) {
        if (s->clients[i].fd >= 0 && s->clients[i].len == 0) {
            return true;
        }
    }
    return false;
}

/* 이미지를 s->enc에 인코딩하고 길이를 반환. *fmt에 0(RGB)/1(JPEG) */
static size_t encode_image(aig_preview_server_t *s, const uint8_t *rgb,
                           uint32_t w, uint32_t h, uint8_t *fmt)
{
#ifdef AIG_HAVE_JPEG
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    unsigned char *out = NULL;
    unsigned long out_len = 0;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_mem_dest(&cinfo, &out, &out_len);
    cinfo.image_width = w;
    cinfo.image_height = h;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, s->quality, TRUE);
    jpeg_start_compress(&cinfo, TRUE);
    while (cinfo.next_scanline < cinfo.image_height) {
        JSAMPROW row = (JSAMPROW)(rgb + (size_t)cinfo.next_scanline * w * 3u);
        jpeg_write_scanlines(&cinfo, &row, 1);
    }
    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    if (out_len > s->enc_cap) {
        uint8_t *nb = (uint8_t *)realloc(s->enc, out_len);
        if (nb == NULL) {
            free(out);
            return 0;
        }
        s->enc = nb;
        s->enc_cap = out_len;
    }
    memcpy(s->enc, out, out_len);
    free(out);
    *fmt = 1;
    return (size_t)out_len;
#else
    size_t n = (size_t)w * h * 3u;
    if (n > s->enc_cap) {
        uint8_t *nb = (uint8_t *)realloc(s->enc, n);
        if (nb == NULL) {
            return 0;
        }
        s->enc = nb;
        s->enc_cap = n;
    }
    memcpy(s->enc, rgb, n);
    *fmt = 0;
    return n;
#endif
}

void aig_preview_send(aig_preview_server_t *s, const char *meta,
                      const uint8_t *rgb, uint32_t img_w, uint32_t img_h)
{
    size_t meta_len = strlen(meta);
    size_t img_len;
    size_t total;
    uint8_t fmt = 0;

    if (!aig_preview_wants_frame(s)) {
        return;
    }
    img_len = encode_image(s, rgb, img_w, img_h, &fmt);
    if (img_len == 0) {
        return;
    }
    total = HEADER_LEN + meta_len + img_len;

    for (int i = 0; i < AIG_PREVIEW_MAX_CLIENTS; i++) {
        aig_preview_client_t *c = &s->clients[i];

        if (c->fd < 0 || c->len > 0) {
            continue;           /* 이전 프레임을 아직 보내는 중 -> 이번 프레임은 건너뜀 */
        }
        if (total > c->cap) {
            uint8_t *nb = (uint8_t *)realloc(c->buf, total);
            if (nb == NULL) {
                continue;
            }
            c->buf = nb;
            c->cap = total;
        }
        memcpy(c->buf, "AIGF", 4);
        put_u32le(c->buf + 4, (uint32_t)meta_len);
        put_u32le(c->buf + 8, (uint32_t)img_len);
        c->buf[12] = fmt;
        c->buf[13] = c->buf[14] = c->buf[15] = 0;
        memcpy(c->buf + HEADER_LEN, meta, meta_len);
        memcpy(c->buf + HEADER_LEN + meta_len, s->enc, img_len);
        c->len = total;
        c->off = 0;
        flush_client(s, i);
    }
}

void aig_preview_close(aig_preview_server_t *s)
{
    for (int i = 0; i < AIG_PREVIEW_MAX_CLIENTS; i++) {
        if (s->clients[i].fd >= 0) {
            close(s->clients[i].fd);
        }
        free(s->clients[i].buf);
        memset(&s->clients[i], 0, sizeof(s->clients[i]));
        s->clients[i].fd = -1;
    }
    s->n_clients = 0;
    free(s->enc);
    s->enc = NULL;
    s->enc_cap = 0;
    if (s->listen_fd >= 0) {
        close(s->listen_fd);
        s->listen_fd = -1;
    }
}
