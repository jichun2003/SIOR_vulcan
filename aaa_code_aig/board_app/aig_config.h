/*
 * 가변 파라미터 모음. aaa_code_1111/config.py를 그대로 옮긴 것 + AI-G 보드 전용 항목.
 *
 * 1111의 주석에 남아있는 현장 실측값(화각, 지연 L, PID 게인)을 기본값으로 가져왔다.
 * !! 단, 지연 L과 PID 게인은 "CPU 추론(약 10fps)" 기준이다. NPU는 yolov8s가 약 15ms라
 *    L이 크게 줄어들므로, 보드에서 --sweep으로 다시 실측해서 넣어야 한다.
 *    (Kp*L <= 0.75 권장이므로 L이 줄면 Kp를 올릴 수 있다)
 */

#ifndef AIG_CONFIG_H
#define AIG_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    AIG_MODE_STANDALONE = 0, /* 보드가 추적+모터 제어까지 전부 (UART로 아두이노) */
    AIG_MODE_DETECT_ONLY     /* 보드는 검출만, 결과를 TCP JSON으로 PC에 보냄 */
} aig_run_mode_t;

/* 카메라 프레임을 NPU 입력(정사각형)으로 줄이는 방법.
 * 하드웨어 스케일러(ScalerResize)는 종횡비를 유지하지 않는다 — 1280x720을 640x640으로
 * 그냥 눌러 담는다. 학습 때는 letterbox(종횡비 유지 + 패딩)를 썼으므로 그림이 달라진다.
 *   STRETCH: 그대로 눌러 담음. 화각 전체를 보지만 표적이 가로로 늘어나 정확도가 떨어질 수 있음.
 *            (Telechips 샘플 앱이 쓰는 방식)
 *   CROP   : 가운데를 정사각형으로 잘라서(720x720) 줄임. 그림은 학습과 비슷하지만
 *            수평 화각이 좁아진다 -> hfov를 crop 비율만큼 줄여야 한다 (아래 주석 참고).
 */
typedef enum {
    AIG_FIT_STRETCH = 0,
    AIG_FIT_CROP
} aig_fit_mode_t;

typedef struct {
    /* --- 네트워크 / 카메라 --- */
    const char *network_dir;   /* net.so / npu_cmd.bin / quantized_network.bin 이 있는 폴더 */
    const char *camera_dev;    /* /dev/video0 (보드에 따라 video2일 수 있음 - board_check.sh로 확인) */
    uint32_t    frame_width;
    uint32_t    frame_height;
    float       conf_threshold;
    int         target_class;   /* 이 클래스 id만 표적으로 인정. -1이면 클래스 무시 */
    float       max_jump_frac;  /* 직전 표적에서 프레임 폭 대비 이 비율보다 멀면 무시. 0이면 끔 */
    aig_fit_mode_t fit_mode;
    int         npu_cluster;    /* 0 = single cluster, 1 = multi cluster */

    /* --- 화각 (2026-11-04 1111 실측값) ---
     * 탐지 좌표는 NPU 입력(정사각형)에서 "원본 프레임 픽셀 좌표"로 되돌린 뒤 쓰므로,
     * fit_mode가 STRETCH든 CROP이든 이 값은 원본 프레임 기준 그대로 두면 된다.
     */
    float hfov_deg;
    float vfov_deg;

    /* --- 서보 / 포탑 --- */
    float pan_center_deg;
    float tilt_center_deg;
    float pan_min_deg, pan_max_deg;
    float tilt_min_deg, tilt_max_deg;
    int   lost_target_hold_frames;
    float pan_sign;   /* 화면 오차 -> 서보각 증가 방향 보정 (+1 / -1) */
    float tilt_sign;
    float pan_zero_deg;
    float tilt_zero_deg;

    /* --- 추적 PID (속도형: 오차[deg] -> 회전속도[deg/s]) --- */
    float kp, ki, kd;
    float deadband_deg;
    float max_rate_deg_s;
    float error_filter_alpha;   /* 오차 EMA (1 = 필터 없음) */
    float d_filter_alpha;
    float integral_limit_deg_s;

    /* --- 2자유도 예측 제어 (tracking_filter.py) --- */
    float latency_s;            /* 전체 지연 L (pan) */
    float latency_tilt_s;       /* tilt 지연. < 0이면 latency_s 사용 */
    float ab_alpha;
    float ab_beta;
    float ff_deadband_deg_s;
    float reacquire_gate_deg;
    int   reacquire_frames;
    float coast_hold_s;
    float coast_max_s;

    /* --- 사격 (전동건 DC모터 1개) --- */
    float fire_duration_s;
    int   motor_speed;          /* 0~255 */

    /* --- 시리얼 (아두이노) --- */
    const char *serial_dev;     /* /dev/ttyUSB0 등. NULL이면 시리얼 없이 동작 */
    int         serial_baud;
    double      serial_send_interval_s;

    /* --- 출력 --- */
    aig_run_mode_t run_mode;
    int         json_port;      /* AIG_MODE_DETECT_ONLY에서 열 TCP 포트 */
    int         preview_port;   /* PC 미리보기 영상 TCP 포트 (0 = 끔). 두 모드 모두 사용 가능 */
    int         preview_scale;  /* 카메라 프레임을 1/n로 줄여 보냄 (2: 1280x720 -> 640x360) */
    float       preview_fps;    /* 미리보기 최대 프레임률 (추적 루프보다 낮게) */
    int         preview_quality;/* JPEG 품질 1~100 (libjpeg 있을 때) */
    const char *log_csv;        /* 프레임별 로그 (analyze_log.py로 읽을 수 있는 형식) */
    bool        verbose;

    /* --- 지연 측정 모드 (--sweep) --- */
    bool  sweep;
    int   sweep_axis;           /* 0 = pan, 1 = tilt */
    float sweep_amp_deg;
    float sweep_period_s;
} aig_config_t;

/* 1111 config.py의 기본값. */
static inline aig_config_t aig_config_default(void)
{
    aig_config_t c;

    c.network_dir = "/home/root/yolov8s_target_quantized";
    c.camera_dev = "/dev/video0";
    c.frame_width = 1280;
    c.frame_height = 720;
    c.conf_threshold = 0.4f;
    c.target_class = 0;     /* best1105_live.pt는 클래스 1개 (0 = target) */
    c.max_jump_frac = 0.35f;
    c.fit_mode = AIG_FIT_STRETCH;
    c.npu_cluster = 0;

    c.hfov_deg = 64.0f;     /* 2026-11-04 pan 지연 측정에서 k=1.067 -> 60*1.067 */
    c.vfov_deg = 34.2f;     /* 2026-11-04 tilt 지연 측정에서 k=1.102 -> 31*1.102 */

    c.pan_center_deg = 90.0f;
    c.tilt_center_deg = 90.0f;
    c.pan_min_deg = 10.0f;
    c.pan_max_deg = 170.0f;
    c.tilt_min_deg = 20.0f;
    c.tilt_max_deg = 160.0f;
    c.lost_target_hold_frames = 15;
    c.pan_sign = 1.0f;
    c.tilt_sign = 1.0f;
    c.pan_zero_deg = 0.0f;
    c.tilt_zero_deg = 0.0f;

    c.kp = 2.5f;
    c.ki = 0.1f;
    c.kd = 0.0f;
    c.deadband_deg = 0.4f;
    c.max_rate_deg_s = 60.0f;
    c.error_filter_alpha = 0.9f;
    c.d_filter_alpha = 0.3f;
    c.integral_limit_deg_s = 10.0f;

    /* 1111(Windows + CPU 추론) 실측값은 pan 0.25 / tilt 0.265였지만, NPU 추론은 ~15ms라
     * L이 훨씬 작다. 아래는 "추론 15ms + 프레임 33ms + 서보 반응"의 어림값이고 실측값이
     * 아니다. --sweep으로 재측정해서 --latency로 넣을 것.
     * (test_control 시뮬레이션: 실제 L 0.08s에 0.25s를 넣으면 RMS 2.05 -> 14.65deg) */
    c.latency_s = 0.12f;
    c.latency_tilt_s = 0.12f;
    c.ab_alpha = 0.7f;
    c.ab_beta = 0.4f;
    c.ff_deadband_deg_s = 4.0f;
    c.reacquire_gate_deg = 10.0f;
    c.reacquire_frames = 3;
    c.coast_hold_s = 0.3f;
    c.coast_max_s = 0.8f;

    c.fire_duration_s = 1.5f;
    c.motor_speed = 255;

    c.serial_dev = "/dev/ttyUSB0";
    c.serial_baud = 9600;
    c.serial_send_interval_s = 0.03;

    c.run_mode = AIG_MODE_STANDALONE;
    c.json_port = 5000;
    c.preview_port = 0;
    c.preview_scale = 2;
    c.preview_fps = 10.0f;
    c.preview_quality = 70;
    c.log_csv = NULL;
    c.verbose = false;

    c.sweep = false;
    c.sweep_axis = 0;
    c.sweep_amp_deg = 15.0f;
    c.sweep_period_s = 3.0f;
    return c;
}

#endif /* AIG_CONFIG_H */
