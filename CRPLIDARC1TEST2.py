import serial
import time
import numpy as np
import matplotlib.pyplot as plt

# 1) 시리얼 & RPLIDAR 설정
PORT      = "COM4"
BAUDRATE  = 460800
TIMEOUT   = 1
CMD_STOP  = b'\xA5\x25'
CMD_SCAN  = b'\xA5\x20'

# 2) 패킷 파싱
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

# 3) 플롯 초기화
plt.ion()
fig = plt.figure(figsize=(6,6))
ax = fig.add_subplot(111, polar=True)
ax.set_ylim(0, 8000)
ax.set_theta_zero_location('N')
ax.set_theta_direction(-1)
ax.set_title("RPLIDAR C1 - Continuous Scan", va='bottom')
scat = None

# 4) X 버튼 클릭 시 실행 중단 플래그
RUNNING = True
def on_close(event):
    global RUNNING
    RUNNING = False

fig.canvas.mpl_connect('close_event', on_close)

def main():
    global RUNNING, scat

    ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
    print(f"[+] Open {PORT} @ {BAUDRATE}bps")

    # 초기화: STOP → SCAN → descriptor → 정지 시리얼 버퍼 → 잡음 드롭
    ser.write(CMD_STOP); time.sleep(0.05)
    ser.reset_input_buffer(); ser.reset_output_buffer()
    ser.write(CMD_SCAN)
    desc = ser.read(7)
    if desc[:2] != b'\xA5\x5A':
        print("[-] Invalid descriptor:", desc.hex())
        return

    time.sleep(0.2)
    ser.reset_input_buffer()
    for _ in range(3):
        ser.read(5)

    HIGH_THRESH = 358.0
    LOW_THRESH  = 2.0
    prev_angle = None
    cycle_data = []

    print("[*] Continuous scan started. Close plot window to stop.")

    try:
        # 한 번만 SCAN 명령
        while RUNNING:
            pkt = ser.read(5)
            parsed = parse_scan_data(pkt)
            if not parsed:
                continue

            angle, dist, q = parsed
            cycle_data.append((angle, dist, q))

            # 0° 경계 감지
            if prev_angle and prev_angle > HIGH_THRESH and angle < LOW_THRESH:
                # 그래프 갱신
                angles = np.array([d[0] for d in cycle_data])
                dists  = np.array([d[1] for d in cycle_data])
                thetas = np.deg2rad(angles)

                if scat:
                    scat.remove()
                scat = ax.scatter(thetas, dists, s=5, c='r')

                plt.draw()
                plt.pause(0.001)

                print(f"\n=== New cycle ▶ Points: {len(cycle_data)} ===")
                cycle_data.clear()

            prev_angle = angle

    except Exception as e:
        print(f"\n[!] Error: {e}")

    finally:
        print("[*] Stopping scan and closing serial port.")
        try:
            ser.write(CMD_STOP)
        except:
            pass
        ser.close()
        plt.ioff()
        plt.close(fig)

if __name__ == "__main__":
    main()