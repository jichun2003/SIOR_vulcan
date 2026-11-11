#include "uart.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

static double monotonic_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static speed_t baud_to_speed(int baud)
{
    switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    default:     return B9600;
    }
}

static void queue_cmd(aig_uart_t *u, const char *s)
{
    int slot;

    pthread_mutex_lock(&u->lock);
    if (u->cmd_count < AIG_UART_CMD_QUEUE) {
        slot = (u->cmd_head + u->cmd_count) % AIG_UART_CMD_QUEUE;
        snprintf(u->cmds[slot], AIG_UART_CMD_LEN, "%s", s);
        u->cmd_count++;
    }
    pthread_cond_signal(&u->wake);
    pthread_mutex_unlock(&u->lock);
}

static void uart_write(aig_uart_t *u, const char *data)
{
    size_t len = strlen(data);
    ssize_t n;

    if (u->dead || u->dummy || u->fd < 0) {
        return;
    }
    n = write(u->fd, data, len);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            /* 버퍼가 찼을 뿐 — 이번 각도는 버린다 (다음 프레임에 최신값을 다시 보냄) */
            return;
        }
        fprintf(stderr, "[uart] 쓰기 실패, 이후 명령 무시: %s\n", strerror(errno));
        u->dead = true;
    }
}

static void *worker(void *arg)
{
    aig_uart_t *u = (aig_uart_t *)arg;
    double last_angle_sent = 0.0;

    for (;;) {
        struct timespec deadline;
        char local_cmds[AIG_UART_CMD_QUEUE][AIG_UART_CMD_LEN];
        int  n_cmds = 0;
        bool have_angles = false;
        int  pan = 0, tilt = 0;
        double now;

        pthread_mutex_lock(&u->lock);
        if (!u->stop && u->cmd_count == 0 && !u->have_angles) {
            clock_gettime(CLOCK_MONOTONIC, &deadline);
            {
                long add_ns = (long)(u->send_interval_s * 1e9);
                deadline.tv_nsec += add_ns;
                deadline.tv_sec += deadline.tv_nsec / 1000000000L;
                deadline.tv_nsec %= 1000000000L;
            }
            pthread_cond_timedwait(&u->wake, &u->lock, &deadline);
        }
        if (u->stop && u->cmd_count == 0) {
            pthread_mutex_unlock(&u->lock);
            break;
        }
        while (u->cmd_count > 0 && n_cmds < AIG_UART_CMD_QUEUE) {
            snprintf(local_cmds[n_cmds], AIG_UART_CMD_LEN, "%s", u->cmds[u->cmd_head]);
            u->cmd_head = (u->cmd_head + 1) % AIG_UART_CMD_QUEUE;
            u->cmd_count--;
            n_cmds++;
        }
        if (u->have_angles) {
            have_angles = true;
            pan = u->pan_deg;
            tilt = u->tilt_deg;
            u->have_angles = false;
        }
        pthread_mutex_unlock(&u->lock);

        for (int i = 0; i < n_cmds; i++) {
            uart_write(u, local_cmds[i]);
        }

        now = monotonic_now();
        if (have_angles && (now - last_angle_sent) >= u->send_interval_s) {
            char buf[48];
            last_angle_sent = now;
            snprintf(buf, sizeof(buf), "s2 %d\ns3 %d\n", pan, tilt);
            uart_write(u, buf);
        }
    }
    return NULL;
}

int aig_uart_open(aig_uart_t *u, const char *dev, int baud, double send_interval_s)
{
    struct termios tio;

    memset(u, 0, sizeof(*u));
    u->fd = -1;
    u->send_interval_s = send_interval_s > 0.0 ? send_interval_s : 0.03;
    pthread_mutex_init(&u->lock, NULL);
    pthread_cond_init(&u->wake, NULL);

    if (dev == NULL) {
        u->dummy = true;
        printf("[uart] 시리얼 없이 동작 (각도/사격 명령을 보내지 않음)\n");
    } else {
        printf("[uart] %s 여는 중...\n", dev);
        u->fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (u->fd < 0) {
            fprintf(stderr, "[uart] %s 열기 실패: %s\n", dev, strerror(errno));
            return -1;
        }
        if (tcgetattr(u->fd, &tio) != 0) {
            fprintf(stderr, "[uart] tcgetattr 실패: %s\n", strerror(errno));
            close(u->fd);
            u->fd = -1;
            return -1;
        }
        cfmakeraw(&tio);
        /* 8N1, 흐름제어 없음, 모뎀 제어선 무시 */
        tio.c_cflag &= ~(unsigned)(CSTOPB | PARENB | CRTSCTS);
        tio.c_cflag |= (CLOCAL | CREAD | CS8);
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;
        cfsetispeed(&tio, baud_to_speed(baud));
        cfsetospeed(&tio, baud_to_speed(baud));
        if (tcsetattr(u->fd, TCSANOW, &tio) != 0) {
            fprintf(stderr, "[uart] tcsetattr 실패: %s\n", strerror(errno));
            close(u->fd);
            u->fd = -1;
            return -1;
        }
        /* 포트를 열면 아두이노가 자동 리셋되므로 부팅을 기다린다 (serial_link.py와 동일 2초) */
        usleep(2000 * 1000);
        tcflush(u->fd, TCIOFLUSH);
        printf("[uart] %s 연결됨 (%d baud)\n", dev, baud);
    }

    if (pthread_create(&u->thread, NULL, worker, u) != 0) {
        fprintf(stderr, "[uart] 스레드 생성 실패\n");
        if (u->fd >= 0) {
            close(u->fd);
            u->fd = -1;
        }
        return -1;
    }
    return 0;
}

void aig_uart_send_angles(aig_uart_t *u, float pan_deg, float tilt_deg)
{
    pthread_mutex_lock(&u->lock);
    u->pan_deg = (int)lroundf(pan_deg);
    u->tilt_deg = (int)lroundf(tilt_deg);
    u->have_angles = true;
    pthread_cond_signal(&u->wake);
    pthread_mutex_unlock(&u->lock);
}

void aig_uart_send_fire(aig_uart_t *u, float duration_s, int motor_speed)
{
    char buf[AIG_UART_CMD_LEN];
    long ms = lroundf(duration_s * 1000.0f);

    if (ms < 1) {
        ms = 1;
    }
    if (motor_speed < 0) {
        motor_speed = 0;
    }
    if (motor_speed > 255) {
        motor_speed = 255;
    }
    snprintf(buf, sizeof(buf), "v%d\nt%ld\n", motor_speed, ms);
    queue_cmd(u, buf);
}

void aig_uart_stop_fire(aig_uart_t *u)
{
    queue_cmd(u, "x\n");
}

void aig_uart_close(aig_uart_t *u)
{
    aig_uart_stop_fire(u);        /* 종료 시 모터가 돌고 있으면 확실히 정지 */
    usleep(100 * 1000);           /* 워커가 정지 명령을 보낼 시간 */

    pthread_mutex_lock(&u->lock);
    u->stop = true;
    pthread_cond_signal(&u->wake);
    pthread_mutex_unlock(&u->lock);
    pthread_join(u->thread, NULL);

    if (u->fd >= 0) {
        close(u->fd);
        u->fd = -1;
    }
    pthread_cond_destroy(&u->wake);
    pthread_mutex_destroy(&u->lock);
}
