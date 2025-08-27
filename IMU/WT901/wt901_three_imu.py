import serial
import threading
import time

# ===== 사용자 설정 =====
PORTS = ["COM11", "COM12", "COM13"]   # IMU 3개의 포트 번호를 여기에 맞게 수정
BAUD = 9600
PRINT_INTERVAL = 0.5  # 초
# =====================

def sum2char(high, low):
    value = (high << 8) | low
    if value >= 32768:  # 2's complement
        value -= 65536
    return value

class IMUReader(threading.Thread):
    def __init__(self, port, baud=BAUD, name=None):
        super().__init__(daemon=True, name=name or f"IMUReader-{port}")
        self.port = port
        self.baud = baud
        self.ser = None
        self.lock = threading.Lock()
        # 최신 값 저장소
        self.acc = {"x": 0.0, "y": 0.0, "z": 0.0, "temp": 0.0}
        self.gyro = {"r": 0.0, "p": 0.0, "y": 0.0, "temp": 0.0}
        self.ang = {"r": 0.0, "p": 0.0, "y": 0.0, "temp": 0.0}
        self.mag = {"x": 0.0, "y": 0.0, "z": 0.0, "temp": 0.0}
        self._stop = False

        # 패킷 파서 상태
        self.packet = bytearray(11)
        self.len_cnt = 0

    def open_serial(self):
        while not self._stop:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
                return
            except Exception:
                # 포트가 아직 안 열렸거나 일시적 에러일 수 있으니 재시도
                time.sleep(0.5)

    def close_serial(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    def stop(self):
        self._stop = True
        self.close_serial()

    def parse_accel(self, data):
        ax = sum2char(data[3], data[2]) / 32768 * 16
        ay = sum2char(data[5], data[4]) / 32768 * 16
        az = sum2char(data[7], data[6]) / 32768 * 16
        t  = sum2char(data[9], data[8]) / 100
        with self.lock:
            self.acc.update({"x": ax, "y": ay, "z": az, "temp": t})

    def parse_gyro(self, data):
        gr = sum2char(data[3], data[2]) / 32768 * 2000
        gp = sum2char(data[5], data[4]) / 32768 * 2000
        gy = sum2char(data[7], data[6]) / 32768 * 2000
        t  = sum2char(data[9], data[8]) / 100
        with self.lock:
            self.gyro.update({"r": gr, "p": gp, "y": gy, "temp": t})

    def parse_angle(self, data):
        r = sum2char(data[3], data[2]) / 32768 * 180
        p = sum2char(data[5], data[4]) / 32768 * 180
        y = sum2char(data[7], data[6]) / 32768 * 180
        t = sum2char(data[9], data[8]) / 100
        with self.lock:
            self.ang.update({"r": r, "p": p, "y": y, "temp": t})

    def parse_geomag(self, data):
        mx = sum2char(data[3], data[2])
        my = sum2char(data[5], data[4])
        mz = sum2char(data[7], data[6])
        t  = sum2char(data[9], data[8]) / 100
        with self.lock:
            self.mag.update({"x": mx, "y": my, "z": mz, "temp": t})

    @staticmethod
    def valid_checksum(pkt: bytearray) -> bool:
        # sum(pkt[0..9]) & 0xFF == pkt[10]
        return (sum(pkt[:10]) & 0xFF) == pkt[10]

    def run(self):
        self.open_serial()
        if not self.ser:
            return

        while not self._stop:
            try:
                if self.ser.in_waiting:
                    byte = self.ser.read(1)
                    if not byte:
                        continue
                    b = byte[0]

                    if self.len_cnt == 0:
                        if b == 0x55:
                            self.packet[0] = b
                            self.len_cnt = 1
                    elif self.len_cnt == 1:
                        if 0x50 <= b <= 0x58:
                            self.packet[1] = b
                            self.len_cnt = 2
                        else:
                            self.len_cnt = 0
                    elif 2 <= self.len_cnt < 11:
                        self.packet[self.len_cnt] = b
                        self.len_cnt += 1
                        if self.len_cnt == 11:
                            # 체크섬 확인
                            if self.valid_checksum(self.packet):
                                order = self.packet[1]
                                if order == 0x51:
                                    self.parse_accel(self.packet)
                                elif order == 0x52:
                                    self.parse_gyro(self.packet)
                                elif order == 0x53:
                                    self.parse_angle(self.packet)
                                elif order == 0x54:
                                    self.parse_geomag(self.packet)
                            # 다음 패킷 준비
                            self.len_cnt = 0
                else:
                    time.sleep(0.001)
            except (serial.SerialException, OSError):
                # 연결 끊김 → 재오픈
                self.close_serial()
                self.open_serial()
            except Exception:
                # 기타 예외는 무시하고 계속
                pass

    # 외부에서 최신 값 읽기
    def get_angles(self):
        with self.lock:
            return self.ang["r"], self.ang["p"], self.ang["y"]

    def get_gyros(self):
        with self.lock:
            return self.gyro["r"], self.gyro["p"], self.gyro["y"]

    def get_accels(self):
        with self.lock:
            return self.acc["x"], self.acc["y"], self.acc["z"]


def main():
    readers = [IMUReader(port=p, baud=BAUD, name=f"IMU-{i+1}") for i, p in enumerate(PORTS)]
    for r in readers:
        r.start()

    last_print = time.time()
    try:
        while True:
            now = time.time()
            if now - last_print >= PRINT_INTERVAL:
                # 각도만 예시 출력 (roll, pitch, yaw). 필요하면 가속도/자이로도 아래처럼 추가.
                lines = []
                for i, r in enumerate(readers, start=1):
                    rr, pp, yy = r.get_angles()
                    lines.append(f"IMU{i}({r.port})  Roll:{rr:7.2f}  Pitch:{pp:7.2f}  Yaw:{yy:7.2f}")
                print(" | ".join(lines))
                last_print = now
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        for r in readers:
            r.stop()
        for r in readers:
            r.join()

if __name__ == "__main__":
    main()
