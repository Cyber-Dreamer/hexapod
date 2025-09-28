import serial
import sys
import time

class GPS:
    def __init__(self, port='/dev/ttyAMA0', baudrate=38400, timeout=1):
        self.port = port
        self.ser = serial.Serial(self.port, baudrate, timeout=timeout)

    @staticmethod
    def nmea_to_decimal(coord, direction):
        if not coord or coord == '':
            return None
        if '.' not in coord:
            return None
        degrees_len = 2 if direction in ['N', 'S'] else 3
        degrees = float(coord[:degrees_len])
        minutes = float(coord[degrees_len:])
        decimal = degrees + minutes / 60
        if direction in ['S', 'W']:
            decimal = -decimal
        return decimal

    def read(self):
        line = self.ser.readline().decode('ascii', errors='replace').strip()
        if line.startswith('$GNGGA'):
            parts = line.split(',')
            if len(parts) > 9:
                lat = self.nmea_to_decimal(parts[2], parts[3])
                lon = self.nmea_to_decimal(parts[4], parts[5])
                alt = parts[9]
                if lat is not None and lon is not None and alt and alt != '':
                    return {'lat': lat, 'lon': lon, 'alt': float(alt)}, line
                return None, line  # No fix, but return the raw line
        return None, None  # Not a GNGGA sentence

if __name__ == '__main__':
    try:
        gps = GPS()
        print(f"Successfully opened serial port: {gps.port}")
    except serial.SerialException as e:
        print(f"Error: Could not open serial port: {e}", file=sys.stderr)
        sys.exit(1)

    print("Reading from GPS. Press Ctrl+C to stop.")
    try:
        while True:
            data, raw_line = gps.read()
            if data:
                print(f"Latitude: {data['lat']}, Longitude: {data['lon']}, Altitude: {data['alt']} m")
                time.sleep(1)
            elif raw_line: # We got a GNGGA sentence, but no fix
                print(f"No fix yet... Raw data: {raw_line}")
                time.sleep(1)
            else:
                # No GNGGA sentence was read in this attempt, loop again quickly
                pass
    except KeyboardInterrupt:
        print("\nStopping GPS reader.")
