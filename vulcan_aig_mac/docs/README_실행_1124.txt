발칸 터렛 AI-G 인지부 실행 묶음 (2026-11-24 수정본)
=====================================================

이 압축 파일 하나로 AI-G 보드에서 인지부(카메라 -> NPU 검출)를 돌린다.
11/23 버전을 받았다면 이 버전으로 바꿔야 한다. 표적을 못 잡던 문제 3가지가 고쳐졌다.
무엇이 바뀌었는지는 변경사항_1124.md 참고.

들어 있는 것
- board_app/            보드 앱 소스 (aig_turret)
- tc-nn-app/            카메라 소스 일부
- npu_include/          NPU 헤더
- vulcan_s16_quantized/ 우리 모델 (YOLOv8s int8, 960 입력)
- setup.sh              [보드] 빌드
- run.sh                [보드] 실행 (PC 미리보기 포함)
- conf_watch.sh         [보드] 실행 (미리보기 없이 인식 여부/confidence만, 발열 적음)
- pc_viewer/            [PC] 미리보기 창 (view_aig.py, config.py)
- pc_tools/             [PC] 진단 도구 (grab_frames.py, eval_csi_int8.py)
- 변경사항_1124.md      이번에 찾은 문제와 해결

준비
- PC는 Ubuntu 22.04 기준. 보드와 PC를 같은 공유기에 연결.
- 보드 주소: 192.168.10.238 (동아리방 공유기, 예약 MAC 16:cb:04:ef:a4:d4) 또는 aig-board.local
- 보드 계정: topst / 비밀번호 topst
- CSI 카메라는 보드 전원을 끈 상태에서 꽂고 부팅한다.
- 보드에 방열판 + 팬 권장 (방열판만으로 장시간 돌리다 멈춘 적 있음)

아래 <보드IP> 는 실제 주소로 바꾼다.


1단계 [PC] zip을 보드로 보내기
    cd ~/다운로드
    scp vulcan_aig_bundle_1124.zip topst@<보드IP>:~/

2단계 [PC -> 보드] 접속해서 풀기 (예전 폴더는 지우고 새로)
    ssh topst@<보드IP>
    rm -rf ~/vulcan_aig_bundle
    python3 -m zipfile -e vulcan_aig_bundle_1124.zip .
    cd vulcan_aig_bundle

3단계 [보드] 빌드 (1~2분, sudo 비밀번호 topst)
    bash setup.sh
  "npu test: Success!" 와 마지막 "완료" 가 나오면 성공.

4단계 [보드] 실행 - 둘 중 하나
  (가) 숫자만 보기 (권장, 뷰어 불필요)
    bash conf_watch.sh
    1초마다 이런 줄이 나온다:
       12s  검출 18/18 프레임 (100%)  class 1   conf 평균 0.59 최대 0.72
    - 검출 n/m : 그 1초 동안 표적을 찾은 프레임 수 / 전체 (약 18~20프레임)
    - class 1  : 표적 (NPU는 표적을 1번으로 부른다. 0번은 배경용 빈 자리)
    - conf     : 확신도 0~1. 0.3 이상이면 표적으로 채택
    40초만 돌리고 자동 종료:  DUR=40 bash conf_watch.sh

  (나) PC에서 영상까지 보기
    bash run.sh
    [PC 새 터미널]
    ~/venvs/aig/bin/python ~/다운로드/vulcan_aig_bundle/pc_viewer/view_aig.py --board <보드IP> --port 5001
    (처음 한 번: sudo apt install -y unzip python3-venv
                 python3 -m venv ~/venvs/aig && ~/venvs/aig/bin/pip install opencv-python numpy
                 cd ~/다운로드 && unzip -o vulcan_aig_bundle_1124.zip)
    미리보기는 보드에 부하(약 10%)와 열을 더한다. 필요할 때만 켠다.

  끄기: Ctrl+C (안 꺼지면 새 창에서 sudo pkill -9 aig_turret)

5단계 [보드] 끝낼 때 (중요)
    sudo shutdown -h now
  20초쯤 기다렸다가 전원을 뽑는다.


기대 결과 (11/24 실측, 3.6 m, 흰 벽 앞 고정 과녁)
- 검출 94~100%, conf 평균 약 0.6, 추론 37.4 ms, 약 18 fps
- 과녁이 없을 때 오검출 0


안 될 때
- "CameraSetConfig(1280x720) 실패" (= "버퍼 크기 협상 실패")
    이전에 띄운 앱이 카메라를 잡고 있다.  sudo pkill -9 aig_turret  후 다시 실행.
    run.sh / conf_watch.sh 는 시작할 때 자동으로 정리한다.
- ssh 안 됨: ssh topst@aig-board.local, 또는 공유기 접속 목록에서 aig-board 찾기
- /dev/video2 없음: 전원 끄고 CSI 케이블 다시 꽂고 부팅
- 앱이 "exit 3" 으로 혼자 꺼짐: 카메라 신호 끊김. 케이블 확인 후 다시 실행
- 보드가 응답 없음 / 매우 뜨거움: 전원 끄고 10분 식힌 뒤 재부팅, 팬 확인
- 실행 기록: vulcan_aig_bundle/perception.csv (run.sh)
