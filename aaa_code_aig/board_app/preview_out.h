/*
 * 보드 카메라 영상을 PC로 보내는 TCP 서버 (--preview-port, 두 모드 모두에서 사용 가능).
 *
 * PC에서 보드가 실제로 보고 있는 화면 + 탐지 박스 + 포탑 상태를 실시간으로 확인하는 용도
 * (pc_control/view_aig.py). 추적 제어에는 쓰지 않는다.
 *
 * 프레임 하나 = 16바이트 헤더 + 메타 JSON + 이미지
 *   헤더: "AIGF"(4) | meta_len uint32 LE | img_len uint32 LE | fmt uint8 | 예약 3바이트
 *   fmt : 0 = RGB24 원시(img_w*img_h*3), 1 = JPEG
 *   메타: {"t":..,"frame_w":1280,"frame_h":720,"img_w":640,"img_h":360,"dets":[..],"best":0,...}
 *         박스 좌표는 카메라 원본 프레임 픽셀 기준 (detect-only JSON과 같다)
 *
 * libjpeg가 있으면(CMake가 자동으로 찾음) JPEG로 압축해서 보낸다. 640x360 기준 약 30~50KB.
 * 없으면 원시 RGB(640x360이면 약 690KB/프레임)라 유선 LAN이 아니면 --preview-scale 4를 권장.
 *
 * 추적 루프를 막지 않는 것이 우선이다: 소켓은 논블로킹이고, 이전 프레임을 아직 다 못 보낸
 * 클라이언트에는 새 프레임을 건너뛴다 (밀린 영상은 버림).
 */

#ifndef AIG_PREVIEW_OUT_H
#define AIG_PREVIEW_OUT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define AIG_PREVIEW_MAX_CLIENTS 4

typedef struct {
    int      fd;
    uint8_t *buf;       /* 보내는 중인 프레임 (헤더+메타+이미지) */
    size_t   cap;
    size_t   len;
    size_t   off;       /* 지금까지 보낸 바이트 */
} aig_preview_client_t;

typedef struct {
    int listen_fd;
    int quality;        /* JPEG 품질 1~100 */
    aig_preview_client_t clients[AIG_PREVIEW_MAX_CLIENTS];
    int n_clients;
    uint8_t *enc;       /* 인코딩 결과 임시 버퍼 */
    size_t   enc_cap;
} aig_preview_server_t;

int  aig_preview_open(aig_preview_server_t *s, int port, int quality);
/* 새 접속 수락 + 밀린 데이터 전송. 매 프레임 호출 */
void aig_preview_poll(aig_preview_server_t *s);
/* 새 프레임을 받을 수 있는 클라이언트가 하나라도 있으면 true (없으면 캡처 복사도 생략) */
bool aig_preview_wants_frame(const aig_preview_server_t *s);
/* rgb: img_w*img_h*3 (RGB 순서). meta: 한 줄 JSON */
void aig_preview_send(aig_preview_server_t *s, const char *meta,
                      const uint8_t *rgb, uint32_t img_w, uint32_t img_h);
void aig_preview_close(aig_preview_server_t *s);
/* 빌드에 JPEG 압축이 들어갔는지 (시작 메시지용) */
bool aig_preview_has_jpeg(void);

#endif /* AIG_PREVIEW_OUT_H */
