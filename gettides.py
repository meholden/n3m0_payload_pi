import urllib, json, datetime
import math

##print data # a dict
##print data['predictions']  # a list
##print (data['predictions'][1]) # a dict
##
##
##print data['predictions'][1]["t"] # a date (unicode string)
##print data['predictions'][1]["v"] # tide level (unicode string)
##
### useful conversions
##print float(data['predictions'][1]["v"]) + 10 # float
##print datetime.datetime.strptime(data['predictions'][0]["t"],"%Y-%m-%d %H:%M")
class tidesfromNOAA(object):
    def __init__(self):
        self.station = 9415143 # default
        self.statName = "Carquinez Strait" # default
        #self.getdata() # default commented for efficiency

    def setStatName(self,stat,name):
        self.station = stat
        self.statName = name
        self.getdata()

    def getdata(self):
        url = "https://tidesandcurrents.noaa.gov/api/datagetter"
        url += "?date=today"
        url += "&station=" +str(self.station)
        url += "&product=predictions&datum=mllw&units=metric"
        url += "&application=holdentechnology.com"
        url += "&time_zone=gmt&application=web_services&format=json"

        response = urllib.urlopen(url)
        data = json.loads(response.read())
        #print data
        self.tidedata = data['predictions'] # a list
        print ("Got 24 hours of tides at " + self.statName + " on " + self.tidedata[0]["t"])
        
    def lookup(self,x):
        ## assumption that time increases monotonically
        # find closest points
        lowtime = None
        lowtide = None
        upptime = datetime.datetime.strptime(self.tidedata[0]["t"],"%Y-%m-%d %H:%M")
        hitide = float(self.tidedata[0]["v"])
        for testtime in self.tidedata:
            lowtime = upptime
            lowtide = hitide
            upptime = datetime.datetime.strptime(testtime["t"],"%Y-%m-%d %H:%M")
            hitide = float(testtime["v"])
            if x < upptime:
                break

        # interpolate
        tide = lowtide + (hitide - lowtide)/(upptime - lowtime).total_seconds()*(x - lowtime).total_seconds()

      #  print lowtime, lowtide
      #  print x, tide
      #  print upptime,hitide
        return tide
    
    def nearestStation(self,lat,lon):
        # manually create list of station numbers, names, lat, lon
        # then we can find the closest one on startup automatically
        # this is a tuple of dictionaries in the python world
        # data is from https://tidesandcurrents.noaa.gov/gmap3/ 
        stationtups = (
                {
                "name" : "Carquinez Strait",
                "station" : 9415143,
                "lat" : 38.05833333,
                "lon" : -122.22333333
                },
                {
                "name" : "Richmond",
                "station" : 9414863,
                "lat" : 37.92333333,
                "lon" : -122.415
                },
                {
                "name" : "Threemile Slough",
                "station" : 9414863,
                "lat" : 38.1066667,
                "lon" : -121.7
                }
            )

        mindist = 1e10 # initialize
        

        for st in stationtups:

            dlat = lat - st["lat"]
            dlon = lon - st["lon"]
            dist =  math.sqrt((dlat*dlat) + (dlon*dlon)) * 1.113195e5  # meters

            if (dist < mindist):
                mindist = dist
                bestst = st
            print ("Station " + st["name"] + " at " + str(dist) + " meters")

        print ("Station " + bestst["name"] + " is closest at " + str(mindist) + " meters")

        self.setStatName(bestst["station"],bestst["name"])
        
        return bestst
                    
        

if __name__ == "__main__":

    ##station = 9415143 # carquinez strait
    station = 9414863 # richmond pier

    
    tides = tidesfromNOAA()
    stat = tides.nearestStation(37.99730,-122.09111)
    #tides.setStatName(9415143,"carquinez") # hack here to directly specify station number
    
    # note that this must be within 24hr of the initialization
    x = datetime.datetime.utcnow()

    tide = tides.lookup(x)

    print ("Right now at " + str(x) + " tide is " + str(tide) + "  meters")
  
    
