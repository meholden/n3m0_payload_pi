"""
n3m0 the autonomous boat payload code 1/2019 MEH
This code implements
data collection to geojson and github
and also Twitter posts for something to do.


"""
import sys
## account passwords and API tokens come from here
# stored one directory up for easier repo organization
#sys.path.append("..") # Adds higher directory to python modules path.
sys.path.append("/home/pi/Desktop/dronekitstuff") # Adds higher directory to python modules path.
from secrets import consumer_key, consumer_secret, access_token, access_token_secret, gitrepo


print "Here we go!"

# Import DroneKit-Python
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil # Needed for command message definitions
from picamera import PiCamera
import time
import math
import requests
import os

#Import Twitter
import twitterstuff

# for github commands
import subprocess

# geojson utilities
import makegeojson

# temperature sensor
#import ds18b20temp

# Speed/Depth/Temp via nmea 0183
import nmea_thread


# class to hold info for courses, states, etc.
class PhotoStuff:
##     ## Photo point (where to take photo)
##     point1 = LocationGlobalRelative(38.0, -122.0, 0)
##


     def __init__(self):
          self.mode=0
          self.message = 'no msg'
          
          self.Photoing = False
          self.time_to_quit = False
          self.twitReady = False
          
          ## current location
          self.lat = 38.06807841429639	
          self.lon = -122.23280310630798
          ## picture data
          self.getpix = False
          self.plat=0
          self.plon=0
          self.pmode='none'
          self.pmsg = 'no message'
          self.camera = PiCamera()
     
     def take_photo(self, w,h, filename):
          print('taking photo')
          print filename
          
          self.camera.resolution = (w, h)
          #self.camera.resolution = (1920, 1080)
          #self.camera.resolution = (640, 480)
          self.camera.start_preview()
          #time.sleep(2)

          self.camera.capture(filename)

          self.camera.stop_preview()
          print('photo taken')

def get_distance_meters(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def distance_to_current_waypoint():
     """
     Gets distance in metres to the current waypoint.
     It returns None for the first waypoint (Home location).
     """
     nextwaypoint=vehicle.commands.next
     if nextwaypoint ==0:
        return None

     missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
     lat=missionitem.x
     lon=missionitem.y
     alt=missionitem.z
     targetWaypointLocation=LocationGlobalRelative(lat,lon,alt)
     distancetopoint = get_distance_meters(vehicle.location.global_frame, targetWaypointLocation)
     return distancetopoint


# for taking photos using RC set. CH6 is the trigger.
# Stores last value so we can detect changes
# Must be stored this way so location_callback can use it.
class manualPhoto: 
     def __init__(self):
          self.lastch6 = vehicle.channels['6']

# geoJson file for storing data
# define variables to be stored here
# uses makegeojson.py
class geoJsonClass:
     def __init__(self):
          self.gjlist=[]
          # (lat,lon), datetime, battery, heading, SOG, Temp, Depth, STW, depth time
          self.dataLabels = makegeojson.geothing(["Latitude(deg)","Longitude(deg)"],
                                                 ["date-time","Battery (Volts)",
                                                  "Heading(deg)","GPS Speed(m/s)",
                                                  "Temperature(C)",
                                                  "Depth(ft)",
                                                  "Boatspeed(kt)",
                                                  "Depth Timestamp"])

# Callback when location has changed. 'value' is the updated value
# Mode changing done here.
# myPhoto.Photoing is True when we are heading for a picture point
# also saves current location into myPhoto variables.
def location_callback(self, attr_name, value):
     #print "Location: ", value
     #tempC = ds18b20temp.read_temp()
     #print tempC

     try:
          if (location_callback.lasttime != nmea.depth.time):
               print("SPEED:"+ str(nmea.speed.kt) + "\tDEPTH:" + str(nmea.depth.ft)
                 + "\tTEMP:" + str(nmea.temperature.degC)
                 + "\tTIME:" + str(nmea.speed.time) + " " + str(nmea.depth.time))

               # (lat,lon), datetime, battery, heading, SOG, Temp, Depth, STW, depth time
               myGJ.gjlist.append(makegeojson.geothing([vehicle.location.global_relative_frame.lon,vehicle.location.global_relative_frame.lat],
                                   [time.strftime("%Y-%m-%d %H:%M:%S "),
                                    vehicle.battery.voltage,
                                    vehicle.heading,
                                    vehicle.groundspeed,
                                    nmea.temperature.degC,
                                    nmea.depth.ft,
                                    nmea.speed.kt,
                                    nmea.depth.time]))
               location_callback.lasttime = nmea.depth.time
     except Exception as e:
          location_callback.lasttime = nmea.depth.time
          print(e)
               

     
##     if (myPhoto.mode != vehicle.commands.next):
##          # we have reached waypoint
##
##          # only take one photo per waypoiont
##          myPhoto.mode = vehicle.commands.next
##          myPhoto.Photoing = True

     ## check  distance to waypoint
     dist = distance_to_current_waypoint()  #myPhoto.get_distance_meters(myPhoto.point1,vehicle.location.global_relative_frame)
     #print "dist=", dist, 'photoing=',myPhoto.Photoing,'wp=',vehicle.commands.next,'mode:',myPhoto.mode

     ## toggle switch on RC for different things...
     if (vehicle.channels['6'] > 1500) and (manpho.lastch6 < 1500):
          dist = 0;  # fool test for "at waypoint"
          myPhoto.Photoing=True
          print("snap")
          tempC = ds18b20temp.read_temp()
          myGJ.gjlist.append(makegeojson.geothing([vehicle.location.global_relative_frame.lon,vehicle.location.global_relative_frame.lat],
                              [time.strftime("%Y-%m-%d %H:%M:%S "),vehicle.battery.voltage,vehicle.heading,vehicle.groundspeed,tempC]))

     # save switch last position.
     manpho.lastch6 = vehicle.channels['6']
     
     # if reached photo point: take photo, return to auto mode.
     if ((dist <= 4.0) and myPhoto.Photoing): # waits until we reach photo point, takes photo
          print "Picture!", dist
          # take photo
          myPhoto.take_photo(1920, 1080,'/home/pi/Desktop/cap.jpg')
          myPhoto.twitReady = True # set flag for main loop
          myPhoto.Photoing = False
          ## store data
          myPhoto.lat = vehicle.location.global_relative_frame.lat
          myPhoto.lon = vehicle.location.global_relative_frame.lon
          myPhoto.message = ("Autonomous tweet in real time from a robot boat underway! Photo from wpt " + str(vehicle.commands.next)
                             + " on:" + time.strftime("%Y-%m-%d %H:%M:%S ")
                        + str(tempC) + "C "
                        + "Autopilot mode:" + str(vehicle.mode.name)+ " Battery: "
                        + str(vehicle.battery.voltage) + "V "
                        + str(vehicle.gps_0) + " "
                        + "Position is (" + str(vehicle.location.global_relative_frame.lat)+ ", "
                        + str(vehicle.location.global_relative_frame.lon)+") "
                        )

 
###########################
# Main Code starts here
###########################

# Connect to the Vehicle.
print("\nConnecting to vehicle")
#vehicle = connect(/dev/ttyACM0, wait_ready=True) # pixhawk usb
#vehicle = connect("/dev/ttyUSB0", wait_ready='armed', baud=57600) # telemetry usb
#vehicle = connect("/dev/ttyUSB0", baud=57600) # telemetry usb
#vehicle = connect("/dev/ttyS0", baud=57600) # telemetry usb
try:
     vehicle = connect("tcp:0.0.0.0:14442",wait_ready=True) # telemetry usb
except:
     print ("you must start mavproxy first...")
     # this code does not work- start using command below yourself
     #os.system("mavproxy.py --out=tcpin:0.0.0.0:14441 --out=tcpin:0.0.0.0:14442")
     #time.sleep(1)
     #vehicle = connect("tcp:0.0.0.0:14442",wait_ready=True) # telemetry usb

print "downloading mission"
try:
     cmds = vehicle.commands
     cmds.download()
     cmds.wait_ready()
     print "done downloading"
except:
     print "error downloading mission"

# Using the ``wait_ready(True)`` waits on :py:attr:`parameters`, :py:attr:`gps_0`,
# :py:attr:`armed`, :py:attr:`mode`, and :py:attr:`attitude`. In practice this usually
# means that all supported attributes will be populated.

# 'parameters'
#vehicle.wait_ready('gps_0','armed','mode','attitude')
try:
     vehicle.wait_ready('gps_0')

     # Get some vehicle attributes (state)
     print "Get some vehicle attribute values:"
     print " GPS: %s" % vehicle.gps_0
     print " Battery: %s" % vehicle.battery
     print " Last Heartbeat: %s" % vehicle.last_heartbeat
     print " Is Armable?: %s" % vehicle.is_armable
     print " System status: %s" % vehicle.system_status.state
     print " Mode: %s" % vehicle.mode.name    # settable

except:
     print "Error waiting for vehicle"

# initialize things

starttime = time.localtime()

myPhoto = PhotoStuff()

manpho = manualPhoto()

myGJ = geoJsonClass();

try:
   port = "/dev/ttyUSB0"

   nmea = nmea_thread.nmeaThread(port, 4800)
   print("Starting up nmea...")
   nmea.start()
except:
     print("Error doing nmea startup")
     nmea.kill = True

twi = twitterstuff.twitterstuff(consumer_key,consumer_secret,access_token,access_token_secret)

print "Sending initial Twiter post"
twi.sendmsg("n3m0 started up, let's go have fun! " + time.strftime("%Y-%m-%d %H:%M:%S "),
               vehicle.location.global_relative_frame.lat,
               vehicle.location.global_relative_frame.lon)

## start time used for github updates etc.
start_str = time.strftime("%Y%m%d_%H%M%S")



# Add a callback `location_callback` for the `global_frame` attribute.
vehicle.add_attribute_listener('location.global_frame', location_callback)
##vehicle.add_attribute_listener('mode', mode_callback)

charge_status = 0 #
time_to_geojson = 10 # seconds

####################
# End of Start code
####################

##########################################################
# Loop, this runs things when not interrupted by callbacks.
##########################################################
while not myPhoto.time_to_quit:
     time.sleep(1)
     
     if (myPhoto.twitReady):
          myPhoto.twitReady=False
          print "tweeting photo"
          twi.sendphoto('/home/pi/Desktop/cap.jpg',myPhoto.message,
                                 vehicle.location.global_relative_frame.lat,
                                 vehicle.location.global_relative_frame.lon)
                                 

     ## periodically write sensor measurements to geojson file
     dtime = time.mktime(time.localtime()) - time.mktime(starttime) # seconds
     ##print "dt is:" + str(dtime)
          
     if (dtime > time_to_geojson):
          time_to_geojson = time_to_geojson + 120
          if (len(myGJ.gjlist) > 0):
               try:
                    print "making file:" + start_str + ".geojson"
                    makegeojson.makegeojson("/home/pi/Desktop/dronekitstuff/data_archive/n3m0/" + start_str+".geojson", myGJ.gjlist, myGJ.dataLabels)
               except:
                    print "Can't make geojson data file right now"

               # update github repo.  Assumes credentials are stored in gitrepo URL or use (git config --global credential.helper cache)
               try:
                    print subprocess.check_output('git add .', shell=True, cwd="/home/pi/Desktop/dronekitstuff/data_archive")
                    print subprocess.check_output('git commit -m "auto commit from n3m0"', shell=True, cwd="/home/pi/Desktop/dronekitstuff/data_archive")
                    print subprocess.check_output('git push ' + gitrepo, shell=True, cwd="/home/pi/Desktop/dronekitstuff/data_archive")
                    # run "git pull" from data_archive directory to sync from web
                    # 3 commands above to sync back to web (add/commit/push)
                    # username is meholden, oath token for password.
               except:
                    print "Can't do github right now"


##########################################################
# End infinite Loop, this runs things when not interrupted by callbacks.
##########################################################
          
# Remove observer - specifying the attribute and previously registered callback function
vehicle.remove_message_listener('location.global_frame', location_callback)
#vehicle.remove_message_listener('mode', mode_callback)

# stop nmea thread
nmea.kill = True

# Close vehicle object before exiting script
vehicle.close()

print("Completed")

