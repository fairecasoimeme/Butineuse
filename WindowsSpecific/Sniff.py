# Win32 Wireshark named pipes example
# Requires Python for Windows and the Python for Windows Extensions:
# http://www.python.org
# http://sourceforge.net/projects/pywin32/

import win32pipe, win32file
import time
import subprocess
import serial
import sys
import time

#open Wireshark, configure pipe interface and start capture (not mandatory, you can also do this manually)
wireshark_cmd=['C:\Program Files\Wireshark\Wireshark.exe', r'-i\\.\pipe\wireshark','-k']
proc=subprocess.Popen(wireshark_cmd)

#create the named pipe \\.\pipe\wireshark
pipe = win32pipe.CreateNamedPipe(
    r'\\.\pipe\wireshark',
    win32pipe.PIPE_ACCESS_OUTBOUND,
    win32pipe.PIPE_TYPE_MESSAGE | win32pipe.PIPE_WAIT,
    1, 65536, 65536,
    300,
    None)

#connect to pipe
win32pipe.ConnectNamedPipe(pipe, None)

#open and read an arbitrary pcap file (file must in same folder than script)
#cf = open(r'AssociatinPriseZigBeeXiaomi.pcap', 'rb')
ser = serial.Serial('COM5', 38400, bytesize=8, parity='N', stopbits=1, timeout=None, xonxoff=0, rtscts=0)

#ser.close() 
#ser.open()

print(ser.name)         # check which port was really used
# ser.write(b'hello')     # write a string
ser.write(b'a')
ser.write(b'a')
ser.write(b'h')

print("Init pipe with pcap for wireshark")

# win32file.WriteFile(pipe, 0xd4); win32file.WriteFile(pipe, 0xc3); win32file.WriteFile(pipe, 0xb2); win32file.WriteFile(pipe, 0xa1);
win32file.WriteFile(pipe, bytes([0xd4, 0xc3, 0xb2, 0xa1]) );

#win32file.WriteFile(pipe, 0x02); win32file.WriteFile(pipe, 0x00);
win32file.WriteFile(pipe, bytes([0x02, 0x00]) );

#win32file.WriteFile(pipe, 0x04); win32file.WriteFile(pipe, 0x00);
win32file.WriteFile(pipe, bytes([0x04, 0x00]) );

#win32file.WriteFile(pipe, 0x00); win32file.WriteFile(pipe, 0x00); win32file.WriteFile(pipe, 0x00); win32file.WriteFile(pipe, 0x00);
win32file.WriteFile(pipe, bytes([0x00, 0x00, 0x00, 0x00]) );

#win32file.WriteFile(pipe, 0x00); win32file.WriteFile(pipe, 0x00); win32file.WriteFile(pipe, 0x00); win32file.WriteFile(pipe, 0x00);
win32file.WriteFile(pipe, bytes([0x00, 0x00, 0x00, 0x00]) );

#win32file.WriteFile(pipe, 0xff); win32file.WriteFile(pipe, 0xff); win32file.WriteFile(pipe, 0x00); win32file.WriteFile(pipe, 0x00);
win32file.WriteFile(pipe, bytes([0xff, 0xff, 0x00, 0x00]) );

# C3 : LINKTYPE_IEEE802_15_4	    195	DLT_IEEE802_15_4	    IEEE 802.15.4 wireless Personal Area Network, with each packet having the FCS at the end of the frame.
# E6 : LINKTYPE_IEEE802_15_4_NOFCS	230	DLT_IEEE802_15_4_NOFCS	IEEE 802.15.4 wireless Personal Area Network, without the FCS at the end of the frame.
#win32file.WriteFile(pipe, 0xc3); win32file.WriteFile(pipe, 0x00); win32file.WriteFile(pipe, 0x00); win32file.WriteFile(pipe, 0x00);
# win32file.WriteFile(pipe, bytes([0xc3, 0x00, 0x00, 0x00]) );
win32file.WriteFile(pipe, bytes([0xe6, 0x00, 0x00, 0x00]) );

ser.flushInput()
ser.flushOutput()
time.sleep(.5)

print("Start loop")
while 1:
    #data = cf.read()
    data = ser.read() # read one byte
    print("{0:x}".format(ord(data))),
    #if data == 136:
    #  print ("\n")
    #sys.stdout.write( "{0:x}".format(ord(data)) )
    #sys.stdout.write( " " )
    # data = ser.read(10)        # read up to ten bytes (timeout)

    #wait 2 second (not mandatory, but this let watching data coming trough the pipe)
    #time.sleep(2)

    #send pcap data trough the pipe
    win32file.WriteFile(pipe, data) 

#then pcap data appears into wireshark

ser.close()             # close port