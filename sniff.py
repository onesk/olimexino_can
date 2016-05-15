import sys
import serial
import datetime
import time

fname = datetime.datetime.now().strftime('%Y_%m_%d__%H_%M_%S.can.log')

stime = time.time()
bytecount = 0
with open(fname, "wb") as lf:
    with serial.Serial(port = '/dev/tty.usbmodem1411',
                       baudrate = 115200,
                       bytesize = 8,
                       parity = serial.PARITY_NONE,
                       stopbits = 1,
                       xonxoff = 0,
                       rtscts = 0,
                       timeout = 200) as sp:
        while True:
            line = sp.readline()
            lf.write(line)
            lf.flush()
            bytecount += len(line) + 1
            speed = bytecount / (time.time() - stime + 1e-7)
            sys.stdout.write("%.2f\n" % speed)
            sys.stdout.flush()
            #sys.stdout.write('.')
            #sys.stdout.flush()
