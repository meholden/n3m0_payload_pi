##{
##  "type": "FeatureCollection",
##  "features": [
##{ "type":"Feature","properties": {"loggertime(s)":"326.27","unixtime(s)":"1502144276.46","wpt":"1","speed(m/s)":"1.60","heading(deg)":"246.47","temperature(c)":"25.81"}, "geometry":{"type": "Point","coordinates": [-122.043213,37.920182]}},
##{ "type":"Feature","properties": {"loggertime(s)":"335.34","unixtime(s)":"1502144276.46","wpt":"2","speed(m/s)":"1.17","heading(deg)":"215.99","temperature(c)":"25.37"},"geometry":{"type": "Point","coordinates": [-122.043302,37.920096]}},
##{ "type":"Feature","properties": {"loggertime(s)":"348.24","unixtime(s)":"1502144276.46","wpt":"3","speed(m/s)":"1.39","heading(deg)":"217.11","temperature(c)":"24.12"},
## "geometry":{"type": "Point","coordinates": [-122.043405,37.919965]}},{ "type":"Feature","properties": {"loggertime(s)":"362.74","unixtime(s)":"1502144276.46","wpt":"4","speed(m/s)":"1.40","heading(deg)":"186.11","temperature(c)":"24.37"},
## "geometry":{"type": "Point","coordinates": [-122.043443,37.919818]}},{ "type":"Feature","properties": {"loggertime(s)":"371.74","unixtime(s)":"1502144276.46","wpt":"5","speed(m/s)":"1.41","heading(deg)":"278.93","temperature(c)":"24.50"},
## "geometry":{"type": "Point","coordinates": [-122.043546,37.919776]}},{ "type":"Feature","properties": {"loggertime(s)":"380.83","unixtime(s)":"1502144276.46","wpt":"6","speed(m/s)":"1.27","heading(deg)":"9.46","temperature(c)":"24.87"},
## "geometry":{"type": "Point","coordinates": [-122.043584,37.919834]}},{ "type":"Feature","properties": {"loggertime(s)":"390.63","unixtime(s)":"1502144276.46","wpt":"7","speed(m/s)":"1.42","heading(deg)":"21.35","temperature(c)":"25.06"},
## "geometry":{"type": "Point","coordinates": [-122.043533,37.919971]}},{ "type":"Feature","properties": {"loggertime(s)":"408.73","unixtime(s)":"1502144410.68","wpt":"8","speed(m/s)":"1.32","heading(deg)":"52.35","temperature(c)":"24.69"},
## "geometry":{"type": "Point","coordinates": [-122.043341,37.920109]}}]}

## loggertime(s)	326.27
## unixtime(s)	        1502144276.46
## wpt	                1
## speed(m/s)	        1.60
## heading(deg)	        246.47
## temperature(c)	25.81

## defines data attributes.  Must have coords, everything else is optional
class geothing(object):  # unixtime=0.0 ,wpt=0, speed=0.0, heading=0.0, temperature=0.0
    def __init__(self,coords=[0,0], properties = [0.0 ,0, 0.0, 0.0, 0.0]):
        self.coords = coords
        self.properties = properties


def makegeojson(filename,geoList,geoLabels):
    print filename
    f = open(filename,"w")
    # header
    f.write('{\n')
    f.write(' "type": "FeatureCollection",\n')
    f.write('  "features": [')
    # each point has coordinates and properties
    # some hassle to put commas etc only between things not on last thing, uses [:-1] indexing
    for pt in geoList[:-1]:
        f.write('\n{ "type":"Feature","properties": \n{')
        pname=iter(geoLabels.properties)
        for pp in pt.properties[:-1]:
            f.write('"'+next(pname)+'":"'+str(pp)+'",')

        f.write('"'+next(pname)+'":"'+str(pt.properties[-1])+'"') # last property
        
        f.write('},\n"geometry":{"type": "Point","coordinates": '+str(pt.coords)+'}},')
        
    # last point    
    f.write('\n{ "type":"Feature","properties": \n{')
    pname=iter(geoLabels.properties)
    for pp in geoList[-1].properties[:-1]:
        f.write('"'+next(pname)+'":"'+str(pp)+'",')

    f.write('"'+next(pname)+'":"'+str(geoList[-1].properties[-1])+'"') # last property
    f.write('},\n"geometry":{"type": "Point","coordinates": '+str(geoList[-1].coords)+'}}]}') # last one

            

if __name__ == "__main__":
    # add data to structure
    dataList=[]
    dataList.append(geothing([-122.043341,37.920109], [1502144276.46, 1, 1.60, 246.47, 25.81]))
    dataList.append(geothing([-123.043341,38.920109], [1502144277.46, 1, 2.60, 247.47, 26.81]))
    dataList.append(geothing([-124.043341,39.920109], [1502144278.46, 1, 3.60, 248.47, 27.81]))

    # save names, units of data attributes
    # must match number of elements in the geothing.properties above.
    dataLabels = geothing(["Latitude(deg)","Longitude(deg)"],["Unixtime(sec)","Wpt","Speed(m/s)","Heading(deg)","Temperature(C)"])

    makegeojson("test.geojson", dataList, dataLabels)


