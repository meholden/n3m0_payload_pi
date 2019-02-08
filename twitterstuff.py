import time
from tweepy.streaming import StreamListener
from tweepy import OAuthHandler
from tweepy import Stream
from tweepy import API

class twitterstuff:

    def __init__(self, ckey, csec, atok, atoksec):
        self.auth = OAuthHandler(ckey, csec)
        self.auth.set_access_token(atok, atoksec)
        self.api = API(self.auth)

    def sendmsg(self,msg,llat,llon):
        self.api.update_status(msg,lat=llat, long=llon)

    def sendphoto(self,filen,msg, llat,llon):
        self.api.update_with_media(filen, status=msg,lat=llat, long=llon)


if __name__ == '__main__':
    print("go")
    consumer_key=""
    consumer_secret=""
    access_token=""
    access_token_secret=""

    print("one")

    print("two")

    twi = twitterstuff(consumer_key,consumer_secret,access_token,access_token_secret)

    filename = '/home/pi/Desktop/cap.jpg'
    twitmessage = 'Hello from n3m0 testing '+ time.strftime("%Y-%m-%d %H:%M:%S")

    twmsg = "testing more:" + time.strftime("%Y-%m-%d %H:%M:%S")
    twi.sendmsg(twmsg, 38.0666109, -122.2296231)



    print("done")
