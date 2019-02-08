"""
n3m0 the autonomous boat payload code 1/2019 MEH
This code implements
data collection to geojson and github
and also Twitter posts for something to do.


"""
import sys
## account passwords and API tokens come from here
# stored one directory up for easier repo organization
sys.path.append("..") # Adds higher directory to python modules path.
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
import ds18b20temp



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
     
     def update_n3m0_location(self):
         ## update the boat location
         r=requests.post('http://sailbot.holdentechnology.com/postlatlon.php',
                         data={'b_no':1,'lat':myPhoto.lat,'lon':myPhoto.lon,
                               'mode':myPhoto.mode,'debug':myPhoto.message})
         #print(r.text)

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

     def post_photo(self,filename, newname):
          print ('posting photo')
          url = 'http://sailbot.holdentechnology.com/upload.php'
          #url = 'http://httpbin.org/post'
          data={'submit':'Submit','name':'fileToUpload','id':'fileToUpload'}
          files = {'fileToUpload': (newname, open(filename, 'rb'))}

          rr = requests.post(url, data=data, files=files)
          print rr.text

          print('photo posted')

          
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
          self.dataLabels = makegeojson.geothing(["Latitude(deg)","Longitude(deg)"],["date-time","Battery (Volts)","Heading(deg)","Speed(m/s)","Temperature(C)"])

# Callback when location has changed. 'value' is the updated value
# Mode changing done here.
# myPhoto.Photoing is True when we are heading for a picture point
# also saves current location into myPhoto variables.
def location_callback(self, attr_name, value):
     #print "Location: ", value
     #tempC = ds18b20temp.read_temp()
     #print tempC
     #myGJ.gjlist.append(makegeojson.geothing([vehicle.location.global_relative_frame.lon,vehicle.location.global_relative_frame.lat],
     #                    [time.strftime("%Y-%m-%d %H:%M:%S "),vehicle.battery.voltage,vehicle.heading,vehicle.groundspeed,tempC]))
     
     if (myPhoto.mode != vehicle.commands.next):
          # save data as we have reached waypoint
          # save: position, date-time (string), voltage, heading, speed, temperature
          tempC = ds18b20temp.read_temp()
          print tempC
          myGJ.gjlist.append(makegeojson.geothing([vehicle.location.global_relative_frame.lon,vehicle.location.global_relative_frame.lat],
                              [time.strftime("%Y-%m-%d %H:%M:%S "),vehicle.battery.voltage,vehicle.heading,vehicle.groundspeed,tempC]))

          # only take one photo per waypoiont
          myPhoto.mode = vehicle.commands.next
          myPhoto.Photoing = True

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
          tempC = ds18b20temp.read_temp()
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
     #print myPhoto.get_distance_meters(myPhoto.point1,vehicle.location.global_relative_frame)
     # getting parameters is a little buggy
     #print "Param: %s" % vehicle.parameters['WP_RADIUS']

     #print "Tx CH:", vehicle.channels
     #if (vehicle.channels['6'] > 1000):
     #     print("snap")
     
     if (myPhoto.twitReady):
          myPhoto.twitReady=False
          print "tweeting photo"
          twi.sendphoto('/home/pi/Desktop/cap.jpg',myPhoto.message,
                                 vehicle.location.global_relative_frame.lat,
                                 vehicle.location.global_relative_frame.lon)
                                 
     myPhoto.message = (time.strftime("%Y-%m-%d %H:%M:%S ")
                        + str(vehicle.mode.name)+ " "
                        + str(vehicle.battery) + " "
                        + str(vehicle.gps_0) + " "
                        + "Pos=("+str(vehicle.location.global_relative_frame.lat)+ ", "
                        + str(vehicle.location.global_relative_frame.lon)+") "
                        + str(distance_to_current_waypoint()) + "m to wpt"
                        + str(vehicle.commands.next) )
     myPhoto.update_n3m0_location()

     dtime = time.mktime(time.localtime()) - time.mktime(starttime) # seconds
     ##print "dt is:" + str(dtime)

     ## periodically write sensor measurements to geojson file
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

##     ## low battery check
##     if ((vehicle.battery.voltage < 10.0) or (dtime > 6000)):
##          if (charge_status==0): # freshly dead battery
##               print " Batt: %s" % vehicle.battery.voltage
##               print "Requesting charger assistance from DAV"
##               ## Send /need and get mission ID (repeat if http response not 200).
##               # start mission, get ID.  Not valid unless response code is 200
##               r = requests.get(urlstring + '/need',params=pl_key)
##               if (r.status_code == 200):
##                    server_msg = r.json()
##                    mId = server_msg["missionId"]
##                    print "Mission ID received:", mId
##                    charge_status = 1 # next one
##                    twi.sendmsg((time.strftime("%Y-%m-%d %H:%M:%S ")
##                            +"Requested charger assistance from DAV Network: ID="
##                              + str(mId)),
##                              vehicle.location.global_relative_frame.lat,
##                              vehicle.location.global_relative_frame.lon)
##                    # otherwise keep trying next iteration
##                    
##     if (charge_status==1):
##          print "Checking charge ready."
##          # Poll status, when status of the mission ID is ready_to_charge send boat back
##          pl_keyMis = pl_key.copy()
##          pl_keyMis.update({"mission_id":mId}) # concatenating here
##          r = requests.get(urlstring + '/status',params=pl_keyMis)
##          server_msg = r.json()
##          if (server_msg["state"] == "ready_to_charge"):
##               charge_status = 2
##               vehicle.commands.next = 1 # first waypoint should jump to dock path
##               print "Charger ready, heading home"
##               twi.sendmsg((time.strftime("%Y-%m-%d %H:%M:%S ")
##                            +"DAV Network charger ready, heading home.  Charging will begin when n3m0 in HOLD mode for safety"),
##                              vehicle.location.global_relative_frame.lat,
##                              vehicle.location.global_relative_frame.lon)
##
##     if (charge_status==2):
##          # wait until boat is in HOLD mode then send /begin
##          print "Waiting for HOLD mode"
##          if (str(vehicle.mode.name).find("HOLD") >= 0):
##               r = requests.get(urlstring + '/begin_charging',params=pl_keyMis)
##               server_msg = r.json()
##               print "Charger state is:", server_msg["state"]
##               if (server_msg["state"] != "ready_to_charge"):
##                    charge_status = 3
##                    twi.sendmsg((time.strftime("%Y-%m-%d %H:%M:%S ")
##                                 +"Charging..."),
##                              vehicle.location.global_relative_frame.lat,
##                              vehicle.location.global_relative_frame.lon)
##
##     if (charge_status==3):
##          r = requests.get(urlstring + '/status',params=pl_keyMis)
##          server_msg = r.json()
##          if (server_msg["state"] != "charging_complete"):
##               print "Still Charging: ", server_msg["state"]
##          else:
##               print "Done charging"
##               vehicle.commands.next=2 # start route over
##               charge_status = 4
##               # turn to auto mode here as well later, charge status to 0 if actually charged
##               twi.sendmsg((time.strftime("%Y-%m-%d %H:%M:%S ")
##                            + "Charging complete, mission restarted!"),
##                              vehicle.location.global_relative_frame.lat,
##                              vehicle.location.global_relative_frame.lon)

     # if time to update geojson file, do that
##########################################################
# End infinite Loop, this runs things when not interrupted by callbacks.
##########################################################
          
# Remove observer - specifying the attribute and previously registered callback function
vehicle.remove_message_listener('location.global_frame', location_callback)
#vehicle.remove_message_listener('mode', mode_callback)


# Close vehicle object before exiting script
vehicle.close()

print("Completed")

