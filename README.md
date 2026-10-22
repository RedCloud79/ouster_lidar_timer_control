# Ouster LiDAR 시간별 동작 제어 및 21시 로봇 재시작 연동

## 1. 목적과 동작 시간

Ouster LiDAR를 낮에는 대기 모드로 전환하고, 야간에는 측정하도록 제어한다. 기존 매일 21시 로봇 재시작 작업에는 LiDAR 상태 확인 절차를 추가한다.

| 시간 — 한국시간 | 동작 |
|---|---|
| 06:00~19:59 | LiDAR `STANDBY`: 모터·레이저 정지 |
| 20:00~다음 날 05:59 | LiDAR `NORMAL`: 측정 동작 |
| 매일 21:00 | LiDAR `NORMAL` 및 센서 `RUNNING` 확인 후 기존 클라이언트 종료·로봇 서비스 재시작 |

- 대기 모드는 전원 차단이 아니다. 통신이 유지되며, 공식 문서상 소비전력은 약 5W이다.
- 이 설정은 ROS 드라이버를 06시에 종료하거나 20시에 새로 시작하지 않는다. 센서 모드를 제어하며, 기존 로봇 서비스 재시작은 21시에 수행한다.
- 매분 현재 시각과 센서 상태를 확인한다. 모드가 일치하면 전환 명령을 보내지 않는다.
- 06시·20시 전환은 해당 분의 타이머 실행, API 응답 및 센서 초기화에 따라 약간 지연될 수 있다.
- PC는 켜져 있어야 하며, PC 시각이 정확해야 한다. Python 스크립트는 PC 시간대와 관계없이 한국시간으로 주야간을 판단한다.
- 낮 시간에 설치·활성화하면 LiDAR 측정이 즉시 중단될 수 있다.

## 2. 구성

| 파일 | 역할 |
|---|---|
| `/usr/local/bin/ouster-schedule.py` | 시간별 모드 전환 및 센서 상태 확인 |
| `/etc/systemd/system/ouster-schedule.service` | LiDAR 스크립트를 일회성 실행 |
| `/etc/systemd/system/ouster-schedule.timer` | 부팅 후 및 매분 서비스 실행 |
| `/etc/systemd/system/irop-robot-restart.service` | 기존 클라이언트 종료 및 로봇 서비스 재시작 |
| `/etc/systemd/system/irop-robot-restart.timer` | 기존 매일 21시 재시작 예약 |
| `/etc/systemd/system/irop-robot-restart.service.d/ouster.conf` | 기존 재시작 작업 전에 LiDAR 확인 조건 추가 |

대상 LiDAR IP: `192.168.1.100`  
대상 로봇 서비스: `irop-robot.service`  
필요 프로그램: Python3, systemd. 수동 조회 명령에는 curl을 사용한다.

## 3. 21시 재시작의 정확한 순서

1. 현재 한국시간이 06:00~19:59인지 확인한다. 낮이면 전체 재시작 작업을 건너뛴다.
2. 야간이면 LiDAR의 `operating_mode`를 조회한다.
3. `NORMAL`이 아니면 `NORMAL` 전환 명령을 보낸다.
4. 설정이 `NORMAL`이고 센서 상태가 `RUNNING`인지 확인한다.
5. 확인에 성공하면 기존 `pkill` 명령을 실행한다.
6. `systemctl restart irop-robot.service`를 실행한다.

**LiDAR 확인 실패 시 kill과 restart를 모두 건너뛴다. kill만 실행하고 restart를 생략하는 구조가 아니다.**

`ExecCondition`은 기존 `ExecStartPre`와 `ExecStart`보다 먼저 실행된다. 스크립트가 종료 코드 1을 반환하면 서비스는 조건 불충족으로 건너뛰며, 일반적인 서비스 실패로 표시되지 않을 수 있다. 원인은 journal 로그에서 확인한다.

기존 21시 타이머의 `Persistent=true`는 유지한다. PC가 꺼져 놓친 예약이 부팅 후 실행될 수 있으므로, 낮 시간 조건 검사로 LiDAR가 낮에 켜지는 것을 막는다. 이 조건은 수동으로 재시작 래퍼 서비스를 실행할 때도 적용된다.

센서 `RUNNING` 확인은 ROS 토픽 수신이나 고온 경고 해제를 보장하지 않는다. UDP 수신 상태와 온도 경고는 별도로 확인한다.

## 4. 설치 및 적용

아래 블록을 순서대로 통째로 복사한다. 명령 중간에 빈 줄을 추가하거나 한 줄씩 분리해 실행하지 않는다.

### 4.1 LiDAR 제어 스크립트

```bash
sudo tee /usr/local/bin/ouster-schedule.py >/dev/null <<'EOF'
#!/usr/bin/python3
import datetime
import json
import sys
import time
import urllib.request

BASE = "http://192.168.1.100/api/v1/sensor"
KST = datetime.timezone(datetime.timedelta(hours=9))

def log(message):
    print(message, flush=True)

def api(path, data=None):
    body = None if data is None else json.dumps(data).encode()
    request = urllib.request.Request(
        BASE + path,
        data=body,
        headers={"Content-Type": "application/json"},
        method="GET" if data is None else "POST",
    )
    with urllib.request.urlopen(request, timeout=5) as response:
        raw = response.read()
    return json.loads(raw) if raw.strip() else None

def apply_mode(target):
    current = api("/config/operating_mode")
    if current != target:
        log(f"LiDAR mode: {current} -> {target}")
        api("/config", {"operating_mode": target})

    expected = "RUNNING" if target == "NORMAL" else "STANDBY"
    deadline = time.monotonic() + 60
    last = "unknown"

    while time.monotonic() < deadline:
        try:
            mode = api("/config/operating_mode")
            status = api("/metadata/sensor_info").get("status")
            last = f"mode={mode}, status={status}"
            if mode == target and status == expected:
                log(f"LiDAR ready: {last}")
                return
        except Exception as error:
            last = str(error)
        time.sleep(2)

    raise RuntimeError(f"LiDAR readiness timeout: {last}")

def main():
    hour = datetime.datetime.now(KST).hour
    daytime = 6 <= hour < 20
    action = sys.argv[1] if len(sys.argv) > 1 else "auto"

    if action == "restart-check":
        if daytime:
            log("06:00-20:00: scheduled robot restart skipped.")
            return 1
        apply_mode("NORMAL")
    elif action == "auto":
        apply_mode("STANDBY" if daytime else "NORMAL")
    else:
        raise ValueError(f"Unknown action: {action}")
    return 0

if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        log(f"ERROR: {error}")
        sys.exit(1)
EOF
```
```bash
sudo chmod +x /usr/local/bin/ouster-schedule.py
```
스크립트의 `BASE`가 센서 IP이다. 장비 IP가 다르면 이 값만 수정한다. 상태 확인은 약 60초 동안 재시도하며, 개별 HTTP 요청 제한시간과 서비스 전체 제한시간도 적용된다.

### 4.2 LiDAR 스케줄 서비스

```bash
sudo tee /etc/systemd/system/ouster-schedule.service >/dev/null <<'EOF'
[Unit]
Description=Apply Ouster LiDAR operating schedule
Wants=network-online.target
After=network-online.target

[Service]
Type=oneshot
ExecStart=/usr/bin/python3 /usr/local/bin/ouster-schedule.py auto
TimeoutStartSec=90
EOF
```
### 4.3 LiDAR 스케줄 타이머

```bash
sudo tee /etc/systemd/system/ouster-schedule.timer >/dev/null <<'EOF'
[Unit]
Description=Check Ouster LiDAR schedule every minute

[Timer]
OnBootSec=30s
OnCalendar=*-*-* *:*:00
AccuracySec=1s
Unit=ouster-schedule.service

[Install]
WantedBy=timers.target
EOF
```
매분 확인하므로 통신 실패 후 다음 실행에서 재시도한다. 부팅 후에도 현재 시간에 맞는 모드를 적용한다. 이미 같은 이름의 스케줄러가 있으면 이 내용으로 교체한다.

### 4.4 기존 21시 서비스에 확인 조건 추가

기존 서비스 원본과 타이머를 수정하지 않고 추가 설정 파일을 생성한다.

```bash
sudo mkdir -p /etc/systemd/system/irop-robot-restart.service.d
```
```bash
sudo tee /etc/systemd/system/irop-robot-restart.service.d/ouster.conf >/dev/null <<'EOF'
[Service]
ExecCondition=/usr/bin/python3 /usr/local/bin/ouster-schedule.py restart-check
TimeoutStartSec=180
EOF
```
### 4.5 설정 적용

```bash
sudo systemctl daemon-reload
sudo systemctl restart ouster-schedule.timer
sudo systemctl enable ouster-schedule.timer
sudo systemctl enable --now irop-robot-restart.timer
sudo systemctl start ouster-schedule.service
```
마지막 명령은 현재 시간에 맞게 LiDAR 모드를 적용한다. 위 명령들은 로봇 재시작을 직접 요청하지 않는다. 다만 기존 21시 타이머에 놓친 예약이 있으면 `Persistent=true`에 따라 실행될 수 있으며, 추가한 조건 검사를 거친다.

## 5. 기존 21시 작업 — 유지되는 내용

아래는 기존 설정을 정리한 참고 내용이다. 이미 존재하는 파일을 다시 생성할 필요는 없다.

### irop-robot-restart.service

```ini
[Unit]
Description=Kill manual ROS clients and restart irop-robot service

[Service]
Type=oneshot
ExecStartPre=-/usr/bin/pkill -KILL -f "[m]arket_state_client.py"
ExecStartPre=-/usr/bin/pkill -KILL -f "[t]eleop_twist_keyboard.py"
ExecStartPre=-/usr/bin/pkill -KILL -f "[r]ostopic echo"
ExecStartPre=-/usr/bin/pkill -KILL -x rviz

ExecStart=/bin/systemctl restart irop-robot.service
```
### irop-robot-restart.timer

```ini
[Unit]
Description=Restart irop-robot service every day at 21:00

[Timer]
OnCalendar=*-*-* 21:00:00
Persistent=true

[Install]
WantedBy=timers.target
```
이 타이머의 `21:00`은 PC의 시스템 시간대를 따른다. 현재 장비는 KST로 확인되었다. 다른 장비에 설치할 때는 `timedatectl`로 `Asia/Seoul`인지 확인한다.

## 6. 적용 확인

### 예약 시간

```bash
systemctl list-timers --all --no-pager | grep -E 'ouster-schedule|irop-robot-restart'
```
- `ouster-schedule.timer`: 다음 분의 확인 일정이 표시된다.
- `irop-robot-restart.timer`: 매일 21시 일정이 표시된다.

### 기존 서비스와 추가 조건

```bash
systemctl --no-pager --full cat irop-robot-restart.service
```
출력 아래에 `ouster.conf`와 `ExecCondition`이 표시되어야 한다.

### LiDAR 제어 로그

```bash
journalctl -u ouster-schedule.service -n 30 --no-pager
```
정상적인 낮 시간 예시:

```text
LiDAR mode: NORMAL -> STANDBY
LiDAR ready: mode=STANDBY, status=STANDBY
```
정상적인 야간 예시:

```text
LiDAR ready: mode=NORMAL, status=RUNNING
```
### 21시 재시작 로그

```bash
journalctl -u irop-robot-restart.service -n 50 --no-pager
```
LiDAR 확인 성공 로그 뒤에 기존 재시작 서비스의 완료 기록이 나타나는지 확인한다. 로봇 서비스 자체의 상태도 함께 확인한다.

```bash
systemctl status irop-robot.service --no-pager
```
### 센서 모드·상태·온도 직접 조회

```bash
curl -sS --max-time 5 http://192.168.1.100/api/v1/sensor/config/operating_mode
curl -sS --max-time 5 http://192.168.1.100/api/v1/sensor/metadata/sensor_info
curl -sS --max-time 5 http://192.168.1.100/api/v1/sensor/telemetry
curl -sS --max-time 5 http://192.168.1.100/api/v1/sensor/alerts
```
## 7. 수동 제어

자동 제어가 켜져 있으면 수동으로 바꾼 모드가 다음 분에 스케줄에 맞게 되돌아갈 수 있다. 임시 수동 운용이 필요하면 먼저 스케줄 타이머를 멈추고 실행 중인 확인 작업을 종료한다.

```bash
sudo systemctl stop ouster-schedule.timer
sudo systemctl stop ouster-schedule.service
```
### 수동 대기 모드

```bash
curl -fSs --max-time 15 -X POST http://192.168.1.100/api/v1/sensor/config -H 'Content-Type: application/json' -d '{"operating_mode":"STANDBY"}'
```
### 수동 정상 모드

```bash
curl -fSs --max-time 15 -X POST http://192.168.1.100/api/v1/sensor/config -H 'Content-Type: application/json' -d '{"operating_mode":"NORMAL"}'
```
스케줄 타이머를 멈춰도 기존 21시 재시작 작업은 계속 예약되어 있으며, 야간에는 LiDAR를 NORMAL로 전환할 수 있다.

### 자동 제어 다시 시작

```bash
sudo systemctl start ouster-schedule.timer
sudo systemctl start ouster-schedule.service
```
### 재시작 조건만 확인

```bash
sudo /usr/bin/python3 /usr/local/bin/ouster-schedule.py restart-check
```
이 명령은 kill이나 로봇 restart를 실행하지 않는다. 야간에는 필요시 센서를 NORMAL로 전환하고 RUNNING을 기다린다. 낮에는 건너뛰기 메시지와 종료 코드 1을 반환한다.

실제 재시작 테스트 명령은 아래와 같다. 조건 통과 시 기존 클라이언트가 종료되고 로봇 서비스가 재시작되므로 작업 가능한 시점에만 실행한다.

```bash
sudo systemctl start irop-robot-restart.service
```
## 8. 자동 제어 해제 및 기존 동작으로 복원

LiDAR 스케줄을 중단하고, 21시 재시작에서 LiDAR 조건만 제거한다. 기존 매일 21시 kill·restart 예약은 유지된다.

```bash
sudo systemctl disable --now ouster-schedule.timer
sudo systemctl stop ouster-schedule.service
sudo rm -f /etc/systemd/system/irop-robot-restart.service.d/ouster.conf
sudo systemctl daemon-reload
```
해제만으로 센서가 자동으로 NORMAL로 바뀌지는 않는다. 필요하면 7절의 수동 NORMAL 명령을 실행한다. 스크립트와 스케줄 서비스·타이머 파일은 재사용을 위해 남겨둔다.

## 9. 문제 확인

| 증상 | 확인할 사항 |
|---|---|
| 20시 이후에도 센서가 대기 중 | 스케줄 서비스 로그, PC 시각, 센서 HTTP 통신 |
| 21시 kill·restart가 모두 실행되지 않음 | 재시작 서비스 로그의 ERROR 또는 낮 시간 건너뛰기 메시지 |
| 서비스가 failed로 표시되지 않았는데 재시작 안 됨 | ExecCondition 불충족은 일반 실패 표시 없이 건너뛸 수 있으므로 journal 확인 |
| 낮에 센서가 다시 NORMAL로 바뀜 | ROS 드라이버나 다른 제어 프로그램이 모드를 다시 설정하는지 확인 |
| NORMAL/RUNNING인데 ROS 토픽이 나오지 않음 | ROS 드라이버 상태 및 UDP 목적지 IP·수신 포트 확인 |
| 야간에 고온 출력 제한이 다시 발생 | 온도·SHOT_LIMITING 경고와 방열 상태 확인. 시간 제어만으로 방열 문제가 해결되지는 않음 |

첫 적용 후에는 낮 시간 STANDBY 유지, 20시 ROS 데이터 자동 수신 복구, 21시 전체 재시작을 각각 확인한다. 드라이버가 센서 재초기화에 자동 복구하지 못하면 드라이버 시작·정지 연동을 별도로 추가해야 한다.

## 10. 적용 당시 확인 결과

2026-10-22 현장 출력 기준:

- 15:51:56: NORMAL → STANDBY 전환 요청.
- 15:52:02: mode=STANDBY, status=STANDBY 확인 및 서비스 성공.
- 기존 21시 타이머 예약 유지 확인.
- 기존 재시작 서비스의 ouster.conf / ExecCondition 반영 확인.
- 당시 오늘 20시 전환과 21시 새 조건을 포함한 재시작은 아직 실행 전이므로, 해당 시간 이후 로그 확인이 필요.

## 참고 문서

- [Ouster Standby Operating Mode](https://docs.ouster.com/sensor-docs/firmware/standby-mode)
- [Ouster Sensor Info API](https://docs.ouster.com/sensor-docs/firmware/4.0/api-reference/ouster-http-api/sensor-metadata/get-sensor-info)
- [systemd.service — ExecCondition](https://www.freedesktop.org/software/systemd/man/257/systemd.service.html)
- [systemd.timer](https://www.freedesktop.org/software/systemd/man/257/systemd.timer.html)

