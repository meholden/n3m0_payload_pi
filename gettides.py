import urllib, json, datetime
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
    def __init__(self,stat):
        self.station = stat
        url = "https://tidesandcurrents.noaa.gov/api/datagetter"
        url += "?date=today"
        url += "&station=" +str(self.station)
        url += "&product=predictions&datum=mllw&units=metric"
        url += "&application=holdentechnology.com"
        url += "&time_zone=gmt&application=web_services&format=json"

        response = urllib.urlopen(url)
        data = json.loads(response.read())
        print data
        self.tidedata = data['predictions'] # a list
        print ("Got 24 hours of tides for " + self.tidedata[0]["t"])
        
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

if __name__ == "__main__":
    ##station = 9415143 # carquinez strait
    station = 9414863 # richmond pier
    tides = tidesfromNOAA(station)
    # note that this must be within 24hr of the initialization
    x = datetime.datetime.utcnow()

    tide = tides.lookup(x)

    print (str(x) + " " + str(tide))
  
    
