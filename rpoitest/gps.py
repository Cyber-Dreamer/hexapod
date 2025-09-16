import serial

def nmea_to_decimal(coord, direction):
    if not coord or coord == '':
        return None
    # NMEA format: ddmm.mmmm for latitude, dddmm.mmmm for longitude
    if '.' not in coord:
        return None
    degrees_len = 2 if direction in ['N', 'S'] else 3
    degrees = float(coord[:degrees_len])
    minutes = float(coord[degrees_len:])
    decimal = degrees + minutes / 60
    if direction in ['S', 'W']:
        decimal = -decimal
    return decimal

ser = serial.Serial('/dev/ttyAMA0', 38400, timeout=1)

while True:
    line = ser.readline().decode('ascii', errors='replace').strip()
    if line.startswith('$GNGGA'):
        parts = line.split(',')
        if len(parts) > 9:
            lat = nmea_to_decimal(parts[2], parts[3])
            lon = nmea_to_decimal(parts[4], parts[5])
            alt = parts[9]
            if lat is not None and lon is not None and alt and alt != '':
                print(f"Latitude: {lat}, Longitude: {lon}, Altitude: {alt} m")
            else:
                print(f"No fix. Raw GNGGA: {line}")