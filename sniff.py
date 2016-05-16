import sys
import serial
import datetime
import time
import base64
import struct

fname = datetime.datetime.now().strftime('%Y_%m_%d__%H_%M_%S.can.log')

stime = time.time()
with open(fname, "wb") as lf:
    with serial.Serial(port = '/dev/tty.usbserial',
                       baudrate = 500000,
                       bytesize = 8,
                       parity = serial.PARITY_NONE,
                       stopbits = 1,
                       xonxoff = 0,
                       rtscts = 0,
                       timeout = 200) as sp:
        while True:
            line = sp.readline()

            if len(line) != 18:
                print '!', len(line)
                continue

            try:
                raw = base64.b64decode(line + "A==")
            except e:
                print e
                continue

            lf.write(struct.pack('<cf13s', '@', time.time() - stime, raw))
            lf.flush()

            sys.stdout.write('.')
            sys.stdout.flush()
