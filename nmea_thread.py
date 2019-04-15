from threading import Thread
import serial
import pynmea2  #sudo python2.7 -m pip install pynmea2
import time

class nmeaThread(Thread):
    class Speed(object):
        def __init__(self,time=0,kt=999):
            self.time=time
            self.kt=kt
    class Depth(object):
        def __init__(self,time=0,ft=999):
            self.time=time
            self.ft=ft
    class Temperature(object):
        def __init__(self,time=0,degC=999):
            self.time=time
            self.degC=degC
            
    def __init__(self, port, baud):
        Thread.__init__(self)
        self.port = port
        self.serialPort = serial.Serial(port, baudrate = baud, timeout = 2)
        self.speed = self.Speed()
        self.depth = self.Depth()
        self.temperature = self.Temperature()
        self.kill = False
        
    def parseGPS(self,nmeastr):
        if nmeastr.find('VHW') >=0:
            msg = pynmea2.parse(nmeastr,check=False)
            self.speed.kt = msg.data[4]
            self.speed.time=time.time()
        if nmeastr.find('DBT') >= 0:
            msg = pynmea2.parse(nmeastr,check=False)
            self.depth.ft = msg.data[0]
            self.depth.time = time.time()
            ##print self.depth.time
        # $YXMTW,22.8,C*
        if nmeastr.find('MTW') >=0:
            msg = pynmea2.parse(nmeastr,check=False)
            self.temperature.degC = msg.data[0]
            self.temperature.time=time.time()
            

    # sometimes the NMEA data has values > 128 which causes a crash
    # Thank you stackoverflow for this function.
    def zap_big(self,input_string):
       if len(input_string) == 0:
         return input_string
       if ord(input_string[0]) > 128:
         return " " + self.zap_big(input_string[1:])
       else:
         return input_string[0] + self.zap_big(input_string[1:])

    
    def run(self):
        reader = pynmea2.NMEAStreamReader()
        while (not self.kill):
            data = self.serialPort.read(self.serialPort.inWaiting())
            # catch errors, random 255 in there sometimes...
            try:
                data = self.zap_big(data)
            except Exception as e:
                print('error fixing errors:')
                print(len(data))
                print(str(e))
                    
            #print data
            try:
                for msg in reader.next(data):
    ##                print("+++")
    ##                print (msg)
    ##                print("---")
                    self.parseGPS(str(msg))
            except Exception as e:
                print("error!=======")
                print(data)
                print("===")
                print(str(e))
                print("=====")
        self.serialPort.close()
        print("nmea off!")

 
if __name__ == '__main__':

    try:
        port = "/dev/ttyUSB0"

        nmea = nmeaThread(port, 4800)
        print("Starting up...")
        nmea.start()

        ii=0
        old_speed=7
        old_depth=8
        while (True):
            if (old_speed != nmea.speed.time):
                print("SPEED, Depth, temp: " + str(nmea.speed.kt) + ", " + str(nmea.depth.ft)
                      + ", " + str(nmea.temperature.degC)
                      + " " + str(nmea.speed.time-old_speed) + " " + str(nmea.depth.time-old_depth))
                old_speed = nmea.speed.time
            if (old_depth != nmea.depth.time):
                print("speed, DEPTH, temp: " + str(nmea.speed.kt) + ", " + str(nmea.depth.ft)
                      + ", " + str(nmea.temperature.degC)
                      + " " + str(nmea.speed.time-old_speed) + " " + str(nmea.depth.time-old_depth))
                old_depth = nmea.depth.time
                ii=ii+1
            #time.sleep(0.5)
                
    except KeyboardInterrupt:
        nmea.kill = True
        print("Goodbye!")
        
