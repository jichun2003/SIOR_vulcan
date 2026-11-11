/*
 * 검출 결과를 PC로 보내는 TCP 서버 (--mode detect-only).
 *
 * 한 줄에 JSON 한 개(개행 구분, NDJSON). PC 쪽은 파이썬 표준 socket 모듈만으로 읽을 수
 * 있다 (pc_control/aig_link.py).
 *
 * Telechips의 Vision Protocol(RTPM, 포트 9998/9999)을 쓰지 않는 이유: 메시지 프레이밍이
 * 공개 문서에 없고 libvisionprotocol 바이너리에 묶여 있어서, 우리가 직접 보드 앱을 쓰는
 * 이상 더 단순하고 확인 가능한 방식이 낫다. RTPM GUI로 눈으로 보고 싶을 때는
 * 기본 앱(tcnnapp -i camera -o rtpm)을 그대로 쓰면 된다.
 *
 * 쓰기는 논블로킹이고, 받는 쪽이 느리면 그 줄은 버린다 (추적 루프를 막지 않는 것이 우선).
 */

#ifndef AIG_JSON_OUT_H
#define AIG_JSON_OUT_H

#include <stdbool.h>

#define AIG_JSON_MAX_CLIENTS 4

typedef struct {
    int listen_fd;
    int clients[AIG_JSON_MAX_CLIENTS];
    int n_clients;
} aig_json_server_t;

int  aig_json_open(aig_json_server_t *s, int port);
/* 새 접속을 받아들인다 (논블로킹). 매 프레임 호출 */
void aig_json_poll(aig_json_server_t *s);
/* 접속한 모든 클라이언트에 한 줄 전송 (개행은 이 함수가 붙인다) */
void aig_json_broadcast(aig_json_server_t *s, const char *line);
void aig_json_close(aig_json_server_t *s);

#endif /* AIG_JSON_OUT_H */
