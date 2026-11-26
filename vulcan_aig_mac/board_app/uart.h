/*
 * 아두이노 포탑 컨트롤러와의 UART 통신. aaa_code_1111/serial_link.py 포팅.
 *
 * 프로토콜 (firmware/model_control/model_control.ino와 동일, 9600 baud):
 *   s2 <deg> = pan  (D3, 포탑 베이스 회전)
 *   s3 <deg> = tilt (D9, 포신 고저각)
 *   v<0-255> = 전동건 모터 PWM 세기
 *   t<ms>    = ms 동안 사격 (펌웨어가 시간이 지나면 스스로 모터를 끔)
 *   x        = 즉시 사격 정지
 *
 * 모든 쓰기는 백그라운드 스레드에서 한다. 메인 루프는 "최신 각도"만 넘겨놓고 바로
 * 돌아오므로, USB가 끊기거나 아두이노가 리셋돼도 추적 루프가 멈추지 않는다.
 * 밀린 각도는 버리고 항상 최신 각도만 보낸다 (각도는 덮어쓰기, 사격 명령은 큐).
 *
 * [2026-11-26] I2C 모드 추가: --port에 /dev/i2c-N을 주면 시리얼 대신 I2C로 보낸다.
 *   우노 = I2C 슬레이브 (기본 주소 0x08, 환경변수 AIG_I2C_ADDR로 변경 가능),
 *   펌웨어는 model_control_i2c.ino. 명령 형식은 시리얼과 똑같고, 한 줄(개행 포함)을
 *   I2C 쓰기 한 번으로 보낸다 (우노 Wire 버퍼 32바이트 이하).
 *   배선: 40핀 3번(SDA) -> 우노 A4, 5번(SCL) -> 우노 A5, GND 공통. 보드 /dev/i2c-1.
 */

#ifndef AIG_UART_H
#define AIG_UART_H

#include <pthread.h>
#include <stdbool.h>

#define AIG_UART_CMD_QUEUE 16
#define AIG_UART_CMD_LEN   32

#define AIG_I2C_ADDR_DEFAULT 0x08   /* 우노 슬레이브 주소 (model_control_i2c.ino의 I2C_ADDR) */
#define AIG_I2C_MAX_TX       32     /* 우노 Wire 수신 버퍼 크기 */

typedef struct {
    int  fd;
    bool dead;      /* 복구 불가 오류 -> 이후 명령 무시 */
    bool dummy;     /* 시리얼 없이 동작 (--no-serial) */
    double send_interval_s;

    /* [2026-11-26] I2C 모드 */
    bool          is_i2c;
    unsigned long i2c_errors;      /* 누적 쓰기 실패 횟수 */
    double        i2c_last_err_print;

    pthread_t       thread;
    pthread_mutex_t lock;
    pthread_cond_t  wake;
    bool            stop;

    bool  have_angles;
    int   pan_deg;
    int   tilt_deg;

    char  cmds[AIG_UART_CMD_QUEUE][AIG_UART_CMD_LEN];
    int   cmd_head;
    int   cmd_count;
} aig_uart_t;

/* dev가 NULL이면 더미 모드(아무것도 보내지 않음). 실패 시 -1 */
int  aig_uart_open(aig_uart_t *u, const char *dev, int baud, double send_interval_s);
void aig_uart_send_angles(aig_uart_t *u, float pan_deg, float tilt_deg);
void aig_uart_send_fire(aig_uart_t *u, float duration_s, int motor_speed);
void aig_uart_stop_fire(aig_uart_t *u);
void aig_uart_close(aig_uart_t *u);

#endif /* AIG_UART_H */
