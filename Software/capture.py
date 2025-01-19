import serial
from datetime import datetime

port = '/dev/cu.usbmodem14401'

filename = datetime.now().strftime('%Y%m%d_%H%M%S.txt')

counter = 0

with open(filename, 'wb') as f:
    with serial.Serial(port, 115200) as ser:
        while True:
            try:
                line = ser.readline()
                f.write(line)
                counter += 1
                if counter % 5 == 0:
                    print(line.strip())
            except KeyboardInterrupt:
                break

print('Done')
