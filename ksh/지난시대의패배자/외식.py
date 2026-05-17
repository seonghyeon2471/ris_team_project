#!/usr/bin/env python3
import os, termios, struct, math, time

# ── 포트 ──────────────────────────────────────
LIDAR_PORT   = '/dev/ttyUSB0'
ARDUINO_PORT = '/dev/ttyUSB1'   # GPIO UART면 /dev/ttyAMA0

# ── 로봇 치수 (m) ─────────────────────────────
ROBOT_WIDTH  = 0.16
LIDAR_OFFSET = 0.075            # 라이다 ~ 로봇 중심
MIN_GAP      = ROBOT_WIDTH + 0.10

# ── 임계값 (m) ────────────────────────────────
OBS_THRESH   = 0.55
STOP_DIST    = 0.22

# ── 속도 ──────────────────────────────────────
MAX_V  = 0.18
MAX_W  = 1.4
KP_W   = 1.3
ANGLE_OFFSET = 0                # 라이다 마운팅 오프셋(도)


def uart_open(port, baud=termios.B115200):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
    a = termios.tcgetattr(fd)
    a[0] = a[1] = a[3] = 0
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    a[4] = a[5] = baud
    a[6][termios.VMIN] = 0
    a[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd

def uart_read(fd, n):
    try:    return os.read(fd, n)
    except: return b''

def uart_write(fd, data):
    os.write(fd, data)


def lidar_reset(fd):
    uart_write(fd, bytes([0xA5, 0x40]))
    time.sleep(2.0)
    termios.tcflush(fd, termios.TCIFLUSH)

def lidar_start(fd):
    termios.tcflush(fd, termios.TCIFLUSH)
    uart_write(fd, bytes([0xA5, 0x20]))
    buf, t = b'', time.time() + 2.0
    while len(buf) < 7 and time.time() < t:
        buf += uart_read(fd, 7 - len(buf))

def lidar_stop(fd):
    uart_write(fd, bytes([0xA5, 0x25]))
    time.sleep(0.1)
    termios.tcflush(fd, termios.TCIFLUSH)

def collect_scan(fd, min_pts=180, timeout=4.0):
    data, buf, first = {}, bytearray(), True
    t = time.time() + timeout

    while time.time() < t:
        chunk = uart_read(fd, 64)
        if chunk:
            buf.extend(chunk)

        while len(buf) >= 5:
            b = buf[:5]
            if (b[0] & 1) == ((b[0] >> 1) & 1):
                del buf[0]; continue

            quality  = b[0] >> 2
            angle    = ((b[1] >> 1) | (b[2] << 7)) / 64.0
            dist_m   = (b[3] | (b[4] << 8)) / 4000.0
            new_scan = bool(b[0] & 1)
            del buf[:5]

            if new_scan:
                if not first and len(data) >= min_pts:
                    return data
                first = False
                data = {}

            if quality == 0 or dist_m == 0 or dist_m > 6.0:
                continue

            a = (angle + ANGLE_OFFSET) % 360
            if a > 180: a -= 360
            k = round(a, 1)
            if k not in data or dist_m < data[k]:
                data[k] = dist_m

    return data


def find_gaps(scan):
    angles = sorted(a for a in scan if -90 <= a <= 90)
    if not angles:
        return []

    gaps, in_gap, ga, gd = [], False, [], []

    def flush():
        if len(ga) < 2: return
        span  = abs(ga[-1] - ga[0])
        avg_d = sum(gd) / len(gd)
        w = 2.0 * avg_d * math.sin(math.radians(span / 2.0))
        if w >= MIN_GAP:
            gaps.append({'ca': (ga[0]+ga[-1])/2.0, 'w': w})

    for a in angles:
        free = (scan[a] - LIDAR_OFFSET) > OBS_THRESH
        if free:
            if not in_gap: in_gap, ga, gd = True, [a], [scan[a]]
            else:          ga.append(a); gd.append(scan[a])
        else:
            if in_gap: flush(); in_gap, ga, gd = False, [], []
    if in_gap: flush()
    return gaps

def best_gap(gaps):
    if not gaps: return None
    return min(gaps, key=lambda g: abs(g['ca']) - g['w'] * 8.0)

def compute(gap, min_lidar):
    ar = math.radians(gap['ca'])
    dr = max(0.0, min(1.0,
        (min_lidar - LIDAR_OFFSET - STOP_DIST) / (OBS_THRESH - STOP_DIST)))
    v = MAX_V * dr * (1.0 - 0.5 * min(abs(ar)/(math.pi/2), 1.0))
    w = max(-MAX_W, min(MAX_W, KP_W * ar))
    return round(v,4), round(w,4)

def send(fd, v, w):
    uart_write(fd, f"{v:.3f},{w:.3f}\n".encode())

def front_min(scan):
    d = [v for a,v in scan.items() if abs(a) <= 20]
    return min(d) if d else float('inf')


def main():
    print("라이다 장애물 회피 시작")

    ard = uart_open(ARDUINO_PORT)
    time.sleep(2.0)
    print(f"아두이노 OK: {ARDUINO_PORT}")

    lid = uart_open(LIDAR_PORT)
    lidar_reset(lid)
    lidar_start(lid)
    print(f"라이다 OK: {LIDAR_PORT}\n")

    pv, pw, err = 0.0, 0.0, 0

    try:
        while True:
            scan = collect_scan(lid)

            if len(scan) < 50:
                err += 1
                print(f"[경고] 포인트 부족({len(scan)}) {err}/3")
                if err >= 3:
                    lidar_stop(lid); lidar_reset(lid); lidar_start(lid); err = 0
                continue
            err = 0

            mfl = front_min(scan)
            mfr = mfl - LIDAR_OFFSET

            if mfr < STOP_DIST:
                if pv or pw: send(ard, 0.0, 0.0); pv = pw = 0.0
                print(f"[긴급정지] {mfr*100:.1f}cm")
                continue

            g = best_gap(find_gaps(scan))
            if g is None:
                v, w = 0.0, MAX_W * 0.55
                print(f"[탐색회전] 갭없음")
            else:
                v, w = compute(g, mfl)
                print(f"전방:{mfr*100:5.1f}cm 갭:{g['ca']:+.1f}° 폭:{g['w']*100:.0f}cm v={v:+.3f} w={w:+.4f}")

            send(ard, v, w)
            pv, pw = v, w

    except KeyboardInterrupt:
        print("\n종료")
    finally:
        send(ard, 0.0, 0.0)
        lidar_stop(lid)
        os.close(lid)
        os.close(ard)

if __name__ == '__main__':
    main()
