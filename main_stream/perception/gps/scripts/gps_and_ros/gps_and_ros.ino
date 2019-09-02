//#define USE_USBCON
#include <SoftwareSerial.h>
#include <ros.h>
#include <core_msgs/Location.h>

SoftwareSerial gpsSerial(10,11);

char c = "";
String str = "";
String targetStr = "GPGGA";

ros::NodeHandle  nh;
core_msgs::Location location;
ros::Publisher location_msg("location_msg", &location);

void setup()
{
  Serial.begin(9600);
  gpsSerial.begin(9600);
  nh.initNode();
  nh.advertise(location_msg);
}

void loop()
{
   if(gpsSerial.available())
    {
      
      c=gpsSerial.read();
      
      if(c == '\n'){
        
        if(targetStr.equals(str.substring(1, 6))){
          
          int first = str.indexOf(",");
          int two = str.indexOf(",", first+1);
          int three = str.indexOf(",", two+1);
          int four = str.indexOf(",", three+1);
          int five = str.indexOf(",", four+1);

          String Lat = str.substring(two+1, three);
          String Long = str.substring(four+1, five);

          String Lat1 = Lat.substring(0, 2);
          String Lat2 = Lat.substring(2);

          String Long1 = Long.substring(0, 3);
          String Long2 = Long.substring(3);

          double LatF = Lat1.toDouble() + Lat2.toDouble()/60;
          float LongF = Long1.toFloat() + Long2.toFloat()/60;

          location.Lat = LatF;
          location.Long = LongF;
          location_msg.publish( &location );
        }

        str = "";

      }else{ 
        str += c;
      }
    }
     nh.spinOnce();
}
