import socket
import threading
import re
import time
import os


#Teleplot
teleplotAddr = ("127.0.0.1",47269)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def sendTelemetry(name, value):
    now = time.time() * 1000
    msg = name+":"+str(now)+":"+str(value)+"|g"
    sock.sendto(msg.encode(), teleplotAddr)

# --- Enums (optional placeholders, since C# used them) ---
class RideType:
    rides = {0: "UnknownRide", 1: "RollerCoaster", 2: "MineTrain"}
    def __getitem__(self, key):
        return self.rides.get(key, "UnknownRide")

class CartType:
    carts = {0: "UnknownCart", 1: "FrontCart", 2: "BackCart"}
    def __getitem__(self, key):
        return self.carts.get(key, "UnknownCart")

class GunType:
    guns = {0: "NoGun", 1: "Laser", 2: "Shotgun"}
    def __getitem__(self, key):
        return self.guns.get(key, "NoGun")

# --- Main Telemetry Receiver ---
class EpicRollerCoastersTelemetry:
    def __init__(self, ip="0.0.0.0", port=7701):
        self.ip = ip
        self.port = port
        self.last_received_udp_packet = ""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.ip, self.port))
        print(f"[INFO] Listening for UDP packets on {self.ip}:{self.port}")

        self.ride_type = RideType()
        self.cart_type = CartType()
        self.gun_type = GunType()

    def start(self):
        thread = threading.Thread(target=self.receive_data, daemon=True)
        thread.start()
        print("[INFO] UDP receive thread started.")
        # Keep main thread alive
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[INFO] Telemetry listener stopped.")

    def receive_data(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
                text = data.decode("utf-8")
                self.last_received_udp_packet = text

                speed = self.get_number("S", text)
                vibration_intensity = self.get_number("V", text)
                vibration_frequency = self.get_number("F", text)
                yaw = self.get_number("Y", text)
                pitch = self.get_number("P", text)
                roll = self.get_number("R", text)
                heave = self.get_number("H", text)
                sway = self.get_number("W", text)
                surge = self.get_number("U", text)
                yaw_vel = self.get_number("A", text)
                pitch_vel = self.get_number("I", text)
                roll_vel = self.get_number("O", text)
                ride_index = self.get_int("N", text)
                car_index = self.get_int("C", text)
                gun_index = self.get_int("G", text)

                ride_name = self.ride_type[ride_index]
                car_name = self.cart_type[car_index]
                gun_name = self.gun_type[gun_index]

                os.system('cls' if os.name == 'nt' else 'clear')
                print(f">> Speed: {speed:.3f}, Yaw: {yaw:.3f}, Pitch: {pitch:.3f}, Roll: {roll:.3f}, "
                      f"Heave: {heave:.3f}, Sway: {sway:.3f}, Surge: {surge:.3f}, "
                      f"YawVelocity: {yaw_vel:.3f}, PitchVelocity: {pitch_vel:.3f}, RollVelocity: {roll_vel:.3f}, "
                      f"RideName: {ride_name}, CarName: {car_name}, GunName: {gun_name}, "
                      f"VibrationIntensity: {vibration_intensity:.3f}, VibrationFrequency: {vibration_frequency:.3f}")
                sendTelemetry("Speed",speed)

                sendTelemetry("Yaw",yaw)
                sendTelemetry("Pitch",pitch)
                sendTelemetry("Roll",roll)

                sendTelemetry("Heave",heave)
                sendTelemetry("Sway",sway)
                sendTelemetry("Surge",surge)

                sendTelemetry("YawVelocity",yaw_vel)
                sendTelemetry("PitchVelocity",pitch_vel)
                sendTelemetry("RollVelocity",roll_vel)

                sendTelemetry("VibrationIntensity",vibration_intensity)

            except Exception as e:
                print(f"[UDPReceive] Error: {e}")

    def get_number(self, prefix, data):
        number_rx = r"-*\d{3}\.\d{3}"
        data_block_rx = rf"{prefix}\[{number_rx}\]"
        match = re.search(data_block_rx, data)
        if match:
            num_match = re.search(number_rx, match.group())
            if num_match:
                return float(num_match.group())
        return 0.0

    def get_int(self, prefix, data):
        int_rx = r"-*\d{2}"
        data_block_rx = rf"{prefix}\[{int_rx}\]"
        match = re.search(data_block_rx, data)
        if match:
            num_match = re.search(int_rx, match.group())
            if num_match:
                return int(num_match.group())
        return 0
1.

if __name__ == "__main__":
    telemetry = EpicRollerCoastersTelemetry(port=7701)
    telemetry.start()
