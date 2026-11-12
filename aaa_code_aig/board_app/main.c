/*
 * SIOR AI-G 보드 앱 진입점.
 * aaa_code_1111/main.py의 2dof 추적 루프를 보드(TOPST AI-G)용 C로 옮긴 것.
 *
 * 두 가지 모드:
 *   --mode standalone   (기본) 카메라 -> NPU -> α-β 예측 -> 2자유도 PID -> UART로 아두이노.
 *                       PC 없이 보드 단독으로 추적/사격한다.
 *   --mode detect-only  검출만 하고 결과를 TCP JSON으로 내보낸다. 제어는 PC의 파이썬이
 *                       담당 (pc_control/main_aig.py). UART는 열지 않는다.
 *
 * 신호 (표준입력에 한 줄 입력):
 *   shot   : 1번째 = 선도각 장입(LEAD), 2번째 = fire_duration초 사격
 *   cancel : 사격 중지 + TRACK 복귀
 *   r      : 중앙 복귀 + 상태 리셋
 *   q      : 종료
 *
 * 지연 측정: --sweep (+ --log-csv). 추적하지 않고 한 축을 사인파로 흔든다.
 *            정지 표적을 화면에 두고 기록한 뒤 analyze_log.py --latency로 L을 계산한다.
 */

#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "aig_config.h"
#include "fire.h"
#include "json_out.h"
#include "npu_vision.h"
#include "preview_out.h"
#include "tracking_filter.h"
#include "turret.h"
#include "uart.h"

static volatile sig_atomic_t g_stop = 0;

static void on_sigint(int sig)
{
    (void)sig;
    g_stop = 1;
}

static double monotonic_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static void usage(const char *prog)
{
    printf(
        "사용법: %s [옵션]\n"
        "  --network <폴더>      net.so/npu_cmd.bin/quantized_network.bin 위치\n"
        "  --camera <dev>        카메라 디바이스 (기본 /dev/video0, 보드에 따라 video2)\n"
        "  --width/--height <n>  카메라 해상도 (기본 1280x720)\n"
        "  --conf <f>            신뢰도 임계값 (기본 0.4)\n"
        "  --fit stretch|crop    NPU 입력으로 줄이는 방법 (기본 stretch)\n"
        "  --mode standalone|detect-only\n"
        "  --port <dev>          아두이노 시리얼 (기본 /dev/ttyUSB0)\n"
        "  --no-serial           시리얼 없이 추적 로직만\n"
        "  --json-port <n>       detect-only에서 열 TCP 포트 (기본 5000)\n"
        "  --preview-port <n>    PC 미리보기 영상 포트 (기본 0=끔, 예 5001). PC: view_aig.py\n"
        "  --preview-scale <n>   영상을 1/n로 줄여 보냄 (기본 2)\n"
        "  --preview-fps <f>     영상 최대 fps (기본 10)\n"
        "  --preview-quality <n> JPEG 품질 (기본 70)\n"
        "  --hfov/--vfov <deg>   화각 (기본 64.0 / 34.2)\n"
        "  --kp/--ki/--kd <f>    PID 게인\n"
        "  --deadband <deg> --max-rate <deg/s> --err-filter <0~1>\n"
        "  --latency <s> --latency-tilt <s>   지연 L (NPU에서 반드시 재측정)\n"
        "  --ab-alpha <f> --ab-beta <f>\n"
        "  --pan-sign/--tilt-sign <+1|-1>     포탑 장착 방향 보정\n"
        "  --fire-duration <s> --motor-speed <0~255>\n"
        "  --log-csv <경로>      프레임별 로그 (analyze_log.py로 읽음)\n"
        "  --sweep --sweep-axis pan|tilt --sweep-amp <deg> --sweep-period <s>\n"
        "  -v, --verbose         매 프레임 상태 출력\n"
        "  -h, --help\n", prog);
}

enum {
    OPT_NETWORK = 1000, OPT_CAMERA, OPT_WIDTH, OPT_HEIGHT, OPT_CONF, OPT_FIT, OPT_MODE,
    OPT_PORT, OPT_NO_SERIAL, OPT_JSON_PORT, OPT_HFOV, OPT_VFOV, OPT_KP, OPT_KI, OPT_KD,
    OPT_DEADBAND, OPT_MAX_RATE, OPT_ERR_FILTER, OPT_LATENCY, OPT_LATENCY_TILT,
    OPT_AB_ALPHA, OPT_AB_BETA, OPT_PAN_SIGN, OPT_TILT_SIGN, OPT_FIRE_DURATION,
    OPT_MOTOR_SPEED, OPT_LOG_CSV, OPT_SWEEP, OPT_SWEEP_AXIS, OPT_SWEEP_AMP,
    OPT_SWEEP_PERIOD, OPT_TARGET_CLASS, OPT_MAX_JUMP,
    OPT_PREVIEW_PORT, OPT_PREVIEW_SCALE, OPT_PREVIEW_FPS, OPT_PREVIEW_QUALITY
};

static int parse_args(int argc, char **argv, aig_config_t *cfg)
{
    static const struct option opts[] = {
        {"network",       required_argument, NULL, OPT_NETWORK},
        {"camera",        required_argument, NULL, OPT_CAMERA},
        {"width",         required_argument, NULL, OPT_WIDTH},
        {"height",        required_argument, NULL, OPT_HEIGHT},
        {"conf",          required_argument, NULL, OPT_CONF},
        {"fit",           required_argument, NULL, OPT_FIT},
        {"mode",          required_argument, NULL, OPT_MODE},
        {"port",          required_argument, NULL, OPT_PORT},
        {"no-serial",     no_argument,       NULL, OPT_NO_SERIAL},
        {"json-port",     required_argument, NULL, OPT_JSON_PORT},
        {"preview-port",  required_argument, NULL, OPT_PREVIEW_PORT},
        {"preview-scale", required_argument, NULL, OPT_PREVIEW_SCALE},
        {"preview-fps",   required_argument, NULL, OPT_PREVIEW_FPS},
        {"preview-quality", required_argument, NULL, OPT_PREVIEW_QUALITY},
        {"hfov",          required_argument, NULL, OPT_HFOV},
        {"vfov",          required_argument, NULL, OPT_VFOV},
        {"kp",            required_argument, NULL, OPT_KP},
        {"ki",            required_argument, NULL, OPT_KI},
        {"kd",            required_argument, NULL, OPT_KD},
        {"deadband",      required_argument, NULL, OPT_DEADBAND},
        {"max-rate",      required_argument, NULL, OPT_MAX_RATE},
        {"err-filter",    required_argument, NULL, OPT_ERR_FILTER},
        {"latency",       required_argument, NULL, OPT_LATENCY},
        {"latency-tilt",  required_argument, NULL, OPT_LATENCY_TILT},
        {"ab-alpha",      required_argument, NULL, OPT_AB_ALPHA},
        {"ab-beta",       required_argument, NULL, OPT_AB_BETA},
        {"pan-sign",      required_argument, NULL, OPT_PAN_SIGN},
        {"tilt-sign",     required_argument, NULL, OPT_TILT_SIGN},
        {"fire-duration", required_argument, NULL, OPT_FIRE_DURATION},
        {"motor-speed",   required_argument, NULL, OPT_MOTOR_SPEED},
        {"log-csv",       required_argument, NULL, OPT_LOG_CSV},
        {"sweep",         no_argument,       NULL, OPT_SWEEP},
        {"sweep-axis",    required_argument, NULL, OPT_SWEEP_AXIS},
        {"sweep-amp",     required_argument, NULL, OPT_SWEEP_AMP},
        {"sweep-period",  required_argument, NULL, OPT_SWEEP_PERIOD},
        {"target-class",  required_argument, NULL, OPT_TARGET_CLASS},
        {"max-jump-frac", required_argument, NULL, OPT_MAX_JUMP},
        {"verbose",       no_argument,       NULL, 'v'},
        {"help",          no_argument,       NULL, 'h'},
        {NULL, 0, NULL, 0}
    };
    int c;

    while ((c = getopt_long(argc, argv, "vh", opts, NULL)) != -1) {
        switch (c) {
        case OPT_NETWORK:       cfg->network_dir = optarg; break;
        case OPT_CAMERA:        cfg->camera_dev = optarg; break;
        case OPT_WIDTH:         cfg->frame_width = (uint32_t)atoi(optarg); break;
        case OPT_HEIGHT:        cfg->frame_height = (uint32_t)atoi(optarg); break;
        case OPT_CONF:          cfg->conf_threshold = (float)atof(optarg); break;
        case OPT_FIT:
            cfg->fit_mode = (strcmp(optarg, "crop") == 0) ? AIG_FIT_CROP : AIG_FIT_STRETCH;
            break;
        case OPT_MODE:
            if (strcmp(optarg, "detect-only") == 0) {
                cfg->run_mode = AIG_MODE_DETECT_ONLY;
            } else if (strcmp(optarg, "standalone") == 0) {
                cfg->run_mode = AIG_MODE_STANDALONE;
            } else {
                fprintf(stderr, "--mode는 standalone 또는 detect-only\n");
                return -1;
            }
            break;
        case OPT_PORT:          cfg->serial_dev = optarg; break;
        case OPT_NO_SERIAL:     cfg->serial_dev = NULL; break;
        case OPT_JSON_PORT:     cfg->json_port = atoi(optarg); break;
        case OPT_PREVIEW_PORT:  cfg->preview_port = atoi(optarg); break;
        case OPT_PREVIEW_SCALE: cfg->preview_scale = atoi(optarg); break;
        case OPT_PREVIEW_FPS:   cfg->preview_fps = (float)atof(optarg); break;
        case OPT_PREVIEW_QUALITY: cfg->preview_quality = atoi(optarg); break;
        case OPT_HFOV:          cfg->hfov_deg = (float)atof(optarg); break;
        case OPT_VFOV:          cfg->vfov_deg = (float)atof(optarg); break;
        case OPT_KP:            cfg->kp = (float)atof(optarg); break;
        case OPT_KI:            cfg->ki = (float)atof(optarg); break;
        case OPT_KD:            cfg->kd = (float)atof(optarg); break;
        case OPT_DEADBAND:      cfg->deadband_deg = (float)atof(optarg); break;
        case OPT_MAX_RATE:      cfg->max_rate_deg_s = (float)atof(optarg); break;
        case OPT_ERR_FILTER:    cfg->error_filter_alpha = (float)atof(optarg); break;
        case OPT_LATENCY:       cfg->latency_s = (float)atof(optarg); break;
        case OPT_LATENCY_TILT:  cfg->latency_tilt_s = (float)atof(optarg); break;
        case OPT_AB_ALPHA:      cfg->ab_alpha = (float)atof(optarg); break;
        case OPT_AB_BETA:       cfg->ab_beta = (float)atof(optarg); break;
        case OPT_PAN_SIGN:      cfg->pan_sign = (float)atof(optarg); break;
        case OPT_TILT_SIGN:     cfg->tilt_sign = (float)atof(optarg); break;
        case OPT_FIRE_DURATION: cfg->fire_duration_s = (float)atof(optarg); break;
        case OPT_MOTOR_SPEED:   cfg->motor_speed = atoi(optarg); break;
        case OPT_LOG_CSV:       cfg->log_csv = optarg; break;
        case OPT_SWEEP:         cfg->sweep = true; break;
        case OPT_SWEEP_AXIS:    cfg->sweep_axis = (strcmp(optarg, "tilt") == 0) ? 1 : 0; break;
        case OPT_SWEEP_AMP:     cfg->sweep_amp_deg = (float)atof(optarg); break;
        case OPT_SWEEP_PERIOD:  cfg->sweep_period_s = (float)atof(optarg); break;
        case OPT_TARGET_CLASS:  cfg->target_class = atoi(optarg); break;
        case OPT_MAX_JUMP:      cfg->max_jump_frac = (float)atof(optarg); break;
        case 'v':               cfg->verbose = true; break;
        case 'h':               usage(argv[0]); return 1;
        default:                usage(argv[0]); return -1;
        }
    }
    return 0;
}

/* 표준입력에서 명령 한 줄을 읽는다 (논블로킹). 없으면 0, 있으면 1 */
static int read_command(char *buf, size_t len)
{
    struct pollfd pfd;
    ssize_t n;

    pfd.fd = STDIN_FILENO;
    pfd.events = POLLIN;
    if (poll(&pfd, 1, 0) <= 0) {
        return 0;
    }
    n = read(STDIN_FILENO, buf, len - 1);
    if (n <= 0) {
        return 0;
    }
    buf[n] = '\0';
    /* 개행/공백 제거 */
    while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r' || buf[n - 1] == ' ')) {
        buf[--n] = '\0';
    }
    return n > 0 ? 1 : 0;
}

static const char *LOG_HEADER =
    "t_mono,timestamp,mode,found,conf,bbox_w,bbox_h,err_pan_deg,err_tilt_deg,"
    "servo_pan,servo_tilt,meas_pan,meas_tilt,est_pan,est_tilt,omega_pan,omega_tilt,"
    "pred_pan,pred_tilt,resid_pan,resid_tilt,filter_reset,outlier,ff_scale,"
    "infer_ms,fire_state,hfov,vfov\n";

int main(int argc, char **argv)
{
    aig_config_t cfg = aig_config_default();
    aig_vision_t *vision = NULL;
    aig_turret_t turret;
    aig_predictive_tracker_t tracker;
    aig_fire_t fire;
    aig_uart_t uart;
    aig_json_server_t json_srv;
    aig_preview_server_t preview_srv;
    bool uart_ok = false;
    bool json_ok = false;
    bool preview_ok = false;
    double next_preview = 0.0;
    FILE *log_fp = NULL;
    aig_det_t dets[AIG_MAX_DETS];
    float last_center[2];
    bool has_last_center = false;
    double prev_time, sweep_t0, last_status = 0.0;
    float fps = 0.0f;
    int rc;

    rc = parse_args(argc, argv, &cfg);
    if (rc != 0) {
        return rc > 0 ? 0 : 1;
    }
    signal(SIGINT, on_sigint);
    signal(SIGTERM, on_sigint);
    signal(SIGPIPE, SIG_IGN);

    vision = aig_vision_open(&cfg);
    if (vision == NULL) {
        fprintf(stderr, "비전 초기화 실패\n");
        return 1;
    }

    aig_turret_init(&turret, &cfg);
    aig_tracker_init(&tracker, &cfg, &turret);
    aig_fire_init(&fire, &cfg);

    if (cfg.run_mode == AIG_MODE_DETECT_ONLY) {
        if (aig_json_open(&json_srv, cfg.json_port) != 0) {
            aig_vision_close(vision);
            return 1;
        }
        json_ok = true;
        printf("[mode] detect-only: 검출만 하고 TCP %d로 내보냅니다 (제어는 PC)\n", cfg.json_port);
    } else {
        if (aig_uart_open(&uart, cfg.serial_dev, cfg.serial_baud, cfg.serial_send_interval_s) != 0) {
            fprintf(stderr, "시리얼 초기화 실패 (--no-serial로 추적 로직만 시험 가능)\n");
            aig_vision_close(vision);
            return 1;
        }
        uart_ok = true;
        {
            float lt = (cfg.latency_tilt_s >= 0.0f) ? cfg.latency_tilt_s : cfg.latency_s;
            printf("[mode] standalone 2dof: L pan=%.3fs tilt=%.3fs, 화각 %.1fx%.1f, "
                   "Kp=%.2f Ki=%.2f Kd=%.2f, 불감대 %.2fdeg, a=%.2f b=%.2f\n",
                   cfg.latency_s, lt, cfg.hfov_deg, cfg.vfov_deg,
                   cfg.kp, cfg.ki, cfg.kd, cfg.deadband_deg, cfg.ab_alpha, cfg.ab_beta);
            printf("[mode] 신호: 표준입력에 shot(1번=선도각, 2번=%.1f초 사격) / cancel / r / q\n",
                   cfg.fire_duration_s);
        }
        if (cfg.sweep) {
            printf("[mode] 지연 측정: %s를 +-%.0f도, 주기 %.1f초로 흔듭니다 (추적 안 함). "
                   "정지 표적을 화면에 두세요\n",
                   cfg.sweep_axis == 1 ? "tilt" : "pan", cfg.sweep_amp_deg, cfg.sweep_period_s);
            if (cfg.log_csv == NULL) {
                printf("[mode] 경고: --log-csv가 없어 측정 결과가 저장되지 않습니다\n");
            }
        }
        aig_uart_send_angles(&uart, turret.pan_angle, turret.tilt_angle);  /* 시작 시 중앙 */
    }

    if (cfg.preview_port > 0) {
        if (cfg.preview_scale < 1) {
            cfg.preview_scale = 1;
        }
        if (aig_preview_open(&preview_srv, cfg.preview_port, cfg.preview_quality) == 0) {
            preview_ok = true;
            printf("[preview] %ux%u, 최대 %.0ffps. PC에서: python view_aig.py --board <보드IP> --port %d\n",
                   cfg.frame_width / (uint32_t)cfg.preview_scale,
                   cfg.frame_height / (uint32_t)cfg.preview_scale, cfg.preview_fps, cfg.preview_port);
        } else {
            printf("[preview] 미리보기 서버를 열지 못해 영상 없이 계속합니다\n");
        }
    }

    if (cfg.log_csv != NULL) {
        log_fp = fopen(cfg.log_csv, "w");
        if (log_fp == NULL) {
            fprintf(stderr, "로그 파일을 열 수 없습니다: %s (%s)\n", cfg.log_csv, strerror(errno));
        } else {
            fputs(LOG_HEADER, log_fp);
        }
    }

    prev_time = monotonic_now();
    sweep_t0 = prev_time;

    while (!g_stop) {
        int n_dets = 0, best;
        float infer_ms = 0.0f;
        double now, dt;
        float err_x_deg = 0.0f, err_y_deg = 0.0f;
        aig_track_info_t info;
        const aig_det_t *tgt = NULL;
        char cmd[64];

        /* 미리보기를 볼 클라이언트가 있고 fps 제한 안이면 이번 캡처에서 프레임을 복사해 둔다 */
        if (preview_ok) {
            aig_preview_poll(&preview_srv);
            /* 캡처 시각은 지금보다 약 한 프레임 뒤이므로 반 프레임 앞당겨 판단 */
            double half_frame = fps > 1.0f ? 0.5 / fps : 0.0;
            if (aig_preview_wants_frame(&preview_srv) && monotonic_now() + half_frame >= next_preview) {
                aig_vision_request_preview(vision, cfg.preview_scale);
            }
        }

        if (aig_vision_capture_detect(vision, dets, AIG_MAX_DETS, &n_dets, &infer_ms) != 0) {
            continue;
        }
        now = monotonic_now();

        {
            float max_jump_px = cfg.max_jump_frac > 0.0f
                                    ? cfg.max_jump_frac * (float)cfg.frame_width : 0.0f;
            best = aig_pick_target(dets, n_dets, cfg.target_class,
                                   has_last_center ? last_center : NULL, max_jump_px);
        }
        if (best >= 0) {
            tgt = &dets[best];
            last_center[0] = tgt->cx;
            last_center[1] = tgt->cy;
            has_last_center = true;
            /* 픽셀 오차 -> 각도 오차 (track_target.py pixel_error_deg) */
            err_x_deg = ((tgt->cx - (float)cfg.frame_width / 2.0f) / ((float)cfg.frame_width / 2.0f))
                        * (cfg.hfov_deg / 2.0f);
            err_y_deg = ((tgt->cy - (float)cfg.frame_height / 2.0f) / ((float)cfg.frame_height / 2.0f))
                        * (cfg.vfov_deg / 2.0f);
        }

        /* --- detect-only: 결과만 내보내고 제어는 하지 않음 --- */
        if (cfg.run_mode == AIG_MODE_DETECT_ONLY) {
            char line[2048];
            int off;

            aig_json_poll(&json_srv);
            off = snprintf(line, sizeof(line),
                           "{\"t\":%.6f,\"infer_ms\":%.2f,\"frame_w\":%u,\"frame_h\":%u,\"dets\":[",
                           now, infer_ms, cfg.frame_width, cfg.frame_height);
            for (int i = 0; i < n_dets && off > 0 && off < (int)sizeof(line); i++) {
                off += snprintf(line + off, sizeof(line) - (size_t)off,
                                "%s{\"cls\":%d,\"conf\":%.4f,\"x1\":%.1f,\"y1\":%.1f,"
                                "\"x2\":%.1f,\"y2\":%.1f}",
                                i ? "," : "", dets[i].cls, dets[i].conf,
                                dets[i].x1, dets[i].y1, dets[i].x2, dets[i].y2);
            }
            if (off > 0 && off < (int)sizeof(line)) {
                snprintf(line + off, sizeof(line) - (size_t)off, "],\"best\":%d}", best);
                aig_json_broadcast(&json_srv, line);
            }
        } else {
            /* --- standalone: 신호 처리 + 추적 제어 --- */
            if (read_command(cmd, sizeof(cmd))) {
                if (strcmp(cmd, "shot") == 0) {
                    aig_fire_event_t ev = aig_fire_signal(&fire, now);
                    if (ev == AIG_FIRE_EVENT_LEAD) {
                        printf("[fire] 선도각 장입 (선도각 계산 미포팅 -> 중앙 추적 유지)\n");
                    } else if (ev == AIG_FIRE_EVENT_FIRE) {
                        aig_uart_send_fire(&uart, cfg.fire_duration_s, cfg.motor_speed);
                        printf("[fire] 사격 시작 (%.1f초, 추적 계속)\n", cfg.fire_duration_s);
                    }
                } else if (strcmp(cmd, "cancel") == 0) {
                    if (aig_fire_cancel(&fire)) {
                        aig_uart_stop_fire(&uart);
                    }
                    printf("[fire] 취소 -> 중앙 추적\n");
                } else if (strcmp(cmd, "r") == 0) {
                    aig_turret_reset(&turret);
                    aig_tracker_reset(&tracker);
                    aig_tracker_record(&tracker, now);
                    aig_fire_cancel(&fire);
                    has_last_center = false;
                    aig_uart_stop_fire(&uart);
                    aig_uart_send_angles(&uart, turret.pan_angle, turret.tilt_angle);
                    printf("[reset] 중앙 복귀\n");
                } else if (strcmp(cmd, "q") == 0) {
                    break;
                }
            }
            if (aig_fire_update(&fire, now)) {
                aig_uart_stop_fire(&uart);   /* 펌웨어도 스스로 끄지만 확실히 한 번 더 */
                printf("[fire] 사격 종료 -> 중앙 추적\n");
            }

            memset(&info, 0, sizeof(info));
            if (cfg.sweep) {
                float swing = cfg.sweep_amp_deg *
                              sinf(2.0f * (float)M_PI * (float)(now - sweep_t0) / cfg.sweep_period_s);
                turret.pan_angle = cfg.pan_center_deg + (cfg.sweep_axis == 0 ? swing : 0.0f);
                turret.tilt_angle = cfg.tilt_center_deg + (cfg.sweep_axis == 1 ? swing : 0.0f);
                aig_uart_send_angles(&uart, turret.pan_angle, turret.tilt_angle);
            } else if (tgt != NULL) {
                aig_tracker_on_measurement(&tracker, now, err_x_deg, err_y_deg, &info);
                aig_uart_send_angles(&uart, turret.pan_angle, turret.tilt_angle);
            } else {
                aig_tracker_on_lost(&tracker, now, &info);
                if (info.ff_scale > 0.0f) {
                    aig_uart_send_angles(&uart, turret.pan_angle, turret.tilt_angle);
                }
                if (aig_turret_handle_lost(&turret)) {
                    has_last_center = false;
                    if (aig_fire_is_firing(&fire)) {
                        aig_uart_stop_fire(&uart);
                        printf("[fire] 표적 상실 -> 사격 중지\n");
                    }
                    aig_fire_cancel(&fire);
                }
            }
        }

        /* --- 로그 --- */
        if (log_fp != NULL) {
            fprintf(log_fp,
                    "%.4f,%.3f,%s,%d,", now, (double)time(NULL),
                    cfg.sweep ? "sweep" : (cfg.run_mode == AIG_MODE_DETECT_ONLY ? "detect" : "2dof"),
                    tgt != NULL ? 1 : 0);
            if (tgt != NULL) {
                fprintf(log_fp, "%.4f,%.1f,%.1f,%.4f,%.4f,",
                        tgt->conf, tgt->w, tgt->h, err_x_deg, err_y_deg);
            } else {
                fprintf(log_fp, ",,,,,");
            }
            fprintf(log_fp, "%.4f,%.4f,", turret.pan_angle, turret.tilt_angle);
            if (info.has_meas || info.ff_scale > 0.0f) {
                fprintf(log_fp, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%.3f,",
                        info.meas_pan, info.meas_tilt, info.est_pan, info.est_tilt,
                        info.omega_pan, info.omega_tilt, info.pred_pan, info.pred_tilt,
                        info.resid_pan, info.resid_tilt, info.filter_reset, info.outlier,
                        info.ff_scale);
            } else {
                fprintf(log_fp, ",,,,,,,,,,,,,");
            }
            fprintf(log_fp, "%.2f,%s,%.2f,%.2f\n",
                    infer_ms, aig_fire_state_name(&fire), cfg.hfov_deg, cfg.vfov_deg);
        }

        /* --- PC 미리보기 영상 (제어가 끝난 뒤라 이번 프레임의 서보각/사격 상태가 같이 간다) --- */
        if (preview_ok) {
            uint32_t pw = 0, ph = 0;
            const uint8_t *img = aig_vision_preview(vision, &pw, &ph);
            if (img != NULL) {
                char meta[2560];
                int off = snprintf(meta, sizeof(meta),
                                   "{\"t\":%.6f,\"frame_w\":%u,\"frame_h\":%u,\"img_w\":%u,\"img_h\":%u,"
                                   "\"infer_ms\":%.2f,\"fps\":%.1f,\"mode\":\"%s\",\"fire\":\"%s\","
                                   "\"pan\":%.2f,\"tilt\":%.2f,\"dets\":[",
                                   now, cfg.frame_width, cfg.frame_height, pw, ph, infer_ms, fps,
                                   cfg.sweep ? "sweep"
                                             : (cfg.run_mode == AIG_MODE_DETECT_ONLY ? "detect-only" : "standalone"),
                                   aig_fire_state_name(&fire), turret.pan_angle, turret.tilt_angle);
                for (int i = 0; i < n_dets && off > 0 && off < (int)sizeof(meta) - 128; i++) {
                    off += snprintf(meta + off, sizeof(meta) - (size_t)off,
                                    "%s{\"cls\":%d,\"conf\":%.4f,\"x1\":%.1f,\"y1\":%.1f,"
                                    "\"x2\":%.1f,\"y2\":%.1f}",
                                    i ? "," : "", dets[i].cls, dets[i].conf,
                                    dets[i].x1, dets[i].y1, dets[i].x2, dets[i].y2);
                }
                if (off > 0 && off < (int)sizeof(meta) - 32) {
                    snprintf(meta + off, sizeof(meta) - (size_t)off, "],\"best\":%d}", best);
                    aig_preview_send(&preview_srv, meta, img, pw, ph);
                }
                {
                    /* 간격을 누적해서 잡는다 (now+간격으로 하면 다음 캡처 시점까지 밀려 fps가 낮아짐) */
                    double period = cfg.preview_fps > 0.0f ? 1.0 / cfg.preview_fps : 0.0;
                    next_preview = (now - next_preview > period) ? now : next_preview + period;
                }
            }
        }

        /* --- FPS / 상태 출력 --- */
        dt = now - prev_time;
        prev_time = now;
        if (dt > 0.0) {
            fps = 0.9f * fps + 0.1f * (float)(1.0 / dt);
        }
        if (cfg.verbose || now - last_status >= 1.0) {
            last_status = now;
            if (log_fp != NULL) {
                fflush(log_fp);     /* 강제 종료돼도 로그가 남도록 */
            }
            if (tgt != NULL) {
                printf("[status] 표적 conf=%.2f err=(%+.1f,%+.1f)deg w=(%+.1f,%+.1f)deg/s "
                       "-> s2 %ld / s3 %ld  %.1ffps (추론 %.1fms)\n",
                       tgt->conf, err_x_deg, err_y_deg, info.omega_pan, info.omega_tilt,
                       lroundf(turret.pan_angle), lroundf(turret.tilt_angle), fps, infer_ms);
            } else {
                printf("[status] 표적 없음 (탐지 %d개) s2 %ld / s3 %ld  %.1ffps (추론 %.1fms)\n",
                       n_dets, lroundf(turret.pan_angle), lroundf(turret.tilt_angle), fps, infer_ms);
            }
        }
    }

    printf("\n종료 중...\n");
    if (uart_ok) {
        aig_uart_stop_fire(&uart);
        aig_uart_close(&uart);
    }
    if (json_ok) {
        aig_json_close(&json_srv);
    }
    if (preview_ok) {
        aig_preview_close(&preview_srv);
    }
    if (log_fp != NULL) {
        fclose(log_fp);
    }
    aig_vision_close(vision);
    return 0;
}
