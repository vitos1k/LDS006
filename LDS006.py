"""
My reverse engeniered python class to read and parse Laser Unit data LDS-006
Typicaly installed in old Ecovacs robot vacuums
Lidar operates on serial protocol 115200 baudrate Data bits 8, Stop bit 1,
Parity None, Flow Control Xon/Xoff
To start Lidar one needs to send command "$startlds$",and to  stop it "$stoplds"
Usually vacuum before starting module sends "$stoplds$stoplds$startlds$"
Hooking to logic analyzer i found that module 
bursts 13.33ms long packets with 6 packs of 22byte data

Data looks something like:
fa e2 4b 75 75 01 02 08 79 01 ba 07 7c 01 6d 07 84 01 ba 07 8e 06
fa e3 4b 75 89 01 57 07 88 88 00 00 88 88 00 00 9d 01 aa 06 f3 06
fa e4 4b 75 88 88 00 00 6e 01 8e 02 65 01 de 01 5f 01 f8 02 4c 07
fa e5 4b 75 5a 01 b3 05 58 01 f1 01 57 01 a7 02 54 01 2a 06 83 06
fa e6 4b 75 53 01 42 06 55 01 b2 03 56 01 de 03 88 88 00 00 8f 06
fa e7 4b 75 33 01 7a 16 63 01 bc 02 6f 01 ee 00 99 99 00 00 17 07
fa e8 dd 74 88 88 00 00 88 88 00 00 3f 01 42 05 37 01 95 06 ad 06
fa e9 dd 74 36 01 8b 08 32 01 b7 08 2d 01 2a 0a 2c 01 0b 0b 95 05
fa ea dd 74 2b 01 be 0a 2b 01 53 0c 29 01 ce 13 29 01 11 0d 07 06
fa eb dd 74 2c 01 d2 0b 2d 01 40 0b 30 01 39 0a 33 01 ab 09 15 06
fa ec dd 74 36 01 98 08 3d 01 bb 06 48 01 05 04 0d 02 90 06 04 06
fa ed dd 74 99 99 00 00 88 88 00 00 88 88 00 00 14 02 8f 06 35 07
fa ee 78 75 12 02 67 06 2c 02 77 01 99 99 00 00 7d 01 0a 02 b8 05
fa ef 78 75 76 01 b2 01 8a 01 ca 00 85 01 5e 01 7e 01 6e 01 28 07
fa f0 78 75 7f 01 93 02 7b 01 e9 03 7b 01 43 09 7d 01 a6 03 43 07
fa f1 78 75 80 01 bc 02 80 01 21 01 86 01 89 00 8c 01 54 00 ab 06
fa f2 78 75 9c 01 73 00 4c 02 c8 04 50 02 c2 04 59 02 a3 04 1d 07
fa f3 78 75 61 02 81 04 88 88 00 00 99 99 00 00 88 88 00 00 14 07


Each pack consists of:
1. 1byte header = 0xFA
2. 1byte index [0xA0-0xF9 = 90 total indexes]
3. 2bytes of RPM data ( *100 ), for each of 6 packs RPM data is equal
4. 16bytes of data 4 pairs for 2bytes each [distances,error/correction]
5. 2bytes CRC sum

16bytes of data = (2byte distance + 2byte correction) * 4 points
Infact i've never found out what are those correction or error values
they are inconsistant and depend on the distance, and they are very jittery,
while distance values are solid

LIDAR does 5 rev per second, so it's RPM is  300rpm, in above data it's 
something around 0x754b = 30027 , 30027/100 = 300.27rpm

For 1 full revolution it takes 200ms, packet = 13.33ms.
Which is 200ms / 13.33ms = 15 packets
Each packet 6 packs, each pack 4 points. 15 * 6 * 4 = 360 points
So we have resolution of 1 degree

this Handler can give you None values because often lidar sends None values
when point is too far or too close 0x7777,0x8888 or 0x9999 in the stream
Also it clips at  around 120mm, so 0x0000 are esentially None too
"""


import serial
import struct
import time
import threading
import queue
from typing import List, Tuple

class LidarPacketHandler:
    # COM for windows or /dev/ttyS for linux
    def __init__(self, port: str = "COM1", baudrate: int = 115200): 
        self.serial = serial.Serial(port, baudrate, timeout=0.1)
        self.packet_queue = queue.Queue()
        self.running = False        
        # Initialize fixed-size array of 360 points: (distance, error)
        # Angles: 0.0°, 1.0°, ..., 360.0°
        self.points = [(0.0,0.0) for _ in range(360)]
        self.lock = threading.Lock()
        self.serial_lock = threading.Lock()  # Lock for serial port access
        
    def start(self):
        """Start reading and processing packets in a separate thread."""
        self.running = True
        self.reader_thread = threading.Thread(target=self._read_packets)
        self.processor_thread = threading.Thread(target=self._process_packets)
        self.reader_thread.start()
        self.processor_thread.start()
        
    def stop(self):
        """Stop reading and processing packets."""
        self.running = False
        self.reader_thread.join()
        self.processor_thread.join()
        self.serial.close()
        
    def send_command(self, command: str, encoding: str = 'ascii') -> bool:
        """
        Send a text command to the LiDAR via the serial port.
        
        Args:
            command (str):The text command to send.
            encoding (str):The encoding to use for the command(default:'ascii')
        
        Returns:
            bool: True if the command was sent successfully, False otherwise.
        """
        try:
            # Encode the command and append a newline if not present
            if not command.endswith('\n'):
                command += '\n'
            command_bytes = command.encode(encoding)
            
            with self.serial_lock:
                if not self.serial.is_open:
                    print("Serial port is not open")
                    return False
                self.serial.write(command_bytes)
                self.serial.flush()  # Ensure the command is sent immediately
            return True
        except (serial.SerialException, UnicodeEncodeError) as e:
            print(f"Failed to send command: {e}")
            return False
    
    def _read_packets(self):
        """Read raw packets from serial port."""
        buffer = bytearray()
        while self.running:
            try:
                with self.serial_lock:
                    if self.serial.in_waiting > 0:
                        buffer.extend(self.serial.read(self.serial.in_waiting))
                    
                while len(buffer) >= 22:
                    # Find header
                    if buffer[0] == 0xFA:
                        packet = buffer[:22]
                        if len(packet) == 22 and self._verify_checksum(packet):
                            self.packet_queue.put(packet)
                            #print(packet)
                        buffer = buffer[22:]
                    else:
                        # Skip until next header
                        buffer = buffer[buffer.find(b'\xFA'):] if b'\xFA' in buffer else bytearray()
            except serial.SerialException:
                print("Serial port error")
                break
            time.sleep(0.001)  # Prevent CPU overload
            
    def _verify_checksum(self, packet: bytes) -> bool:
        """Verify packet checksum."""
        calculated = sum(packet[:20]) & 0xFFFF
        received = struct.unpack('<H', packet[20:22])[0]
        return calculated == received
    
    def _process_packets(self):
        """Process packets and update fixed-size point array."""        
        packets_per_rotation = 90 # 6 * 15 per revolution
        
        while self.running:
            try:
                packet = self.packet_queue.get(timeout=0.1)
                header, seq_num = packet[0], packet[1]
                
                if header != 0xFA or not (0xA0 <= seq_num <= 0xF9):
                    continue
                    
                # Extract data
                rpm = struct.unpack('<H', packet[2:4])[0] / 100.0  
                # TODO use RPM data to calculate angle based
                # on timings and angular speed

                distances = struct.unpack('<HHHHHHHH', packet[4:20])
                
                # Calculate base index (each packet pair covers 4 points, 4 degrees)
                packet_idx = (seq_num - 0xA0) % packets_per_rotation
                base_point_idx = packet_idx  * 4  # Each packet covers 4 points
                with self.lock:
                    for i in range(4):
                        point_idx = (base_point_idx + i) % 360  # Ensure index wraps around
                        distance = distances[i*2]
                        errorval = distances[i*2+1]    # error correction data? or maybe some sort of checksum 
                        if distance in [0x7777,0x8888,0x9999,0]:
                            distance = None
                            errorval = None
                        self.points[point_idx] = (distance,errorval)   # angle data is index
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Processing error: {e}")
                
    def get_points(self) -> List[Tuple[float, float]]:
        """Get current point cloud (angle in degrees, distance in mm)."""
        with self.lock:
            return self.points.copy()

if __name__ == "__main__":
    # Example usage
    lidar = LidarPacketHandler(port='COM6', baudrate=115200)
    try:
        lidar.start()
        # Example: Send a command to the LiDAR
        lidar.send_command("$startlds$")  # Comand to start lidar
        while True:
            points = lidar.get_points()
            print(f"Points collected: {len(points)}")
            if points:
                if points[90][0]:
                    print(f"Sample point: angle=90°, distance={points[90][0]:.1f}mm, error = {points[90][1]:.1f}")
            time.sleep(0.2)
    except KeyboardInterrupt:
        # Send a stop command before shutting down
        lidar.send_command("$stoplds")
        lidar.stop()
        print("Stopped")