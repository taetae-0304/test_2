import serial
import time
import numpy as np

# NumPy print 설정: 거리값을 소수점 둘째 자리까지 고정 폭으로 표시
np.set_printoptions(formatter={
    'float_kind': lambda x: f"{x:7.2f}"
})

# 시리얼 & RPLIDAR 설정
PORT      = "COM4"
BAUDRATE  = 460800
TIMEOUT   = 1
CMD_STOP  = b'\xA5\x25'
CMD_SCAN  = b'\xA5\x20'

def parse_scan_data(pkt: bytes):
    if len(pkt) != 5:
        return None
    b0 = pkt[0]
    quality      = b0 >> 2
    raw_angle    = ((pkt[2] << 8) | pkt[1]) >> 1
    raw_distance = (pkt[4] << 8) | pkt[3]
    angle    = raw_angle / 64.0
    distance = raw_distance / 4.0
    return angle, distance, quality

def get_one_cycle(ser):
    prev_angle = None
    cycle = []  # [(angle:int, distance:float, quality:int), ...]

    while True:
        pkt = ser.read(5)
        parsed = parse_scan_data(pkt)
        if not parsed:
            continue

        angle, dist, q = parsed
        # 요청하신 타입 변환
        angle_i = int(round(angle))
        dist_f  = round(dist, 2)
        q_i     = int(q)

        cycle.append((angle_i, dist_f, q_i))

        # 360°→0° 전환 감지
        if prev_angle is not None and (prev_angle - angle) > 300:
            break
        prev_angle = angle

    # 구조화된 배열 생성
    dtype = np.dtype([
        ('angle',    np.int32),
        ('distance', np.float32),
        ('quality',  np.int32)
    ])
    return np.array(cycle, dtype=dtype)

def main():
    ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
    # 초기화: STOP → SCAN → descriptor → 정지 시리얼 버퍼 → 잡음 드롭
    ser.write(CMD_STOP); time.sleep(0.05)
    ser.reset_input_buffer(); ser.reset_output_buffer()
    ser.write(CMD_SCAN)
    desc = ser.read(7)
    if desc[:2] != b'\xA5\x5A':
        print("Invalid descriptor:", desc.hex())
        return

    time.sleep(0.2)
    ser.reset_input_buffer()
    for _ in range(3):
        ser.read(5)

    print("[*] 한 사이클 데이터 수집 중… (Ctrl+C 로 종료)")

    try:
        while True:
            data = get_one_cycle(ser)  # shape = (N, ) structured-array
            count = data.shape[0]

            print(f"\n=== New cycle ▶ Points: {count} ===")
            print("[ angle (°), dist (mm), quality ]")
            print(data)

    except KeyboardInterrupt:
        pass
    finally:
        ser.write(CMD_STOP)
        ser.close()

if __name__ == "__main__":
    main()
    # This is a test branch edit
    # This is a test branch edit
