#include <Adafruit_GPS.h>
#include "MPU9250.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
//MPU9250 mpu;
#include <TinyGPS.h>
TinyGPS gps;

MPU9250 mpu;
// what's the name of the hardware serial port?
#define GPSSerial Serial3

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();            
ros::NodeHandle  nh;

float LATITUDE = 0.0;
float LONGITUDE = 0.0;
boolean newGPSdata = false;
boolean newIMUdata = false;
float latitude_gps = 0;
float longitude_gps = 0;
std_msgs::Float32 latitude_msg;
std_msgs::Float32 longitude_msg;
std_msgs::Float32 yaw_msg;
sensor_msgs::Imu imu_msg;
std_msgs::Header header;

ros::Publisher lat_pub("/my_agv/latitude", &latitude_msg);
ros::Publisher lon_pub("/my_agv/longitude", &longitude_msg);
ros::Publisher yaw_pub("/my_agv/yaw", &yaw_msg);
ros::Publisher imu_pub("/my_agv/imu", &imu_msg);
uint32_t prev_ms = millis();
void setup() {

  while (!Serial);  // uncomment to have the sketch wait until Serial is ready
  pinMode(7,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(5,OUTPUT);
  
  nh.initNode();
  nh.advertise(lat_pub);
  nh.advertise(lon_pub);
  nh.advertise(imu_pub);
  nh.advertise(yaw_pub);
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  //MPU stuff
  Wire.begin();
  delay(2000);
  
  if (!mpu.setup(0x68)) {  // change to your own address
      while (1) {
          Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
          delay(5000);
      }
  }
  
  mpu.setAccBias(33.65, 60.52, 44.74);
  mpu.setGyroBias(0.53, 0.31, -0.53);
  mpu.setMagBias(-232.31, -33.16, -638.21);
  mpu.setMagScale(1.01, 0.97, 1.02);
  mpu.setMagneticDeclination(4.03333);

}
void send_data_imu()
{
  header.frame_id = "base_imu";
  imu_msg.header = header;
  imu_msg.orientation.x = mpu.getQuaternionX();
  imu_msg.orientation.y = mpu.getQuaternionY();
  imu_msg.orientation.z = mpu.getQuaternionZ();
  imu_msg.orientation.w = mpu.getQuaternionW();
  imu_msg.linear_acceleration.x = mpu.getAccX()*9.81;
  imu_msg.linear_acceleration.y = mpu.getAccY()*9.81;
  imu_msg.linear_acceleration.z = mpu.getAccZ()*9.81;
  imu_msg.angular_velocity.x = mpu.getGyroX()*0.0174533;
  imu_msg.angular_velocity.y = mpu.getGyroX()*0.0174533;
  imu_msg.angular_velocity.z = mpu.getGyroX()*0.0174533;
  imu_pub.publish( &imu_msg );

  yaw_msg.data = mpu.getYaw();
  yaw_pub.publish(&yaw_msg);
}
void loop()
{
  
  while (GPSSerial.available())
  {
    char c = GPSSerial.read();
    if (gps.encode(c))newGPSdata = true;
  }
  
   if (millis() - timer > 1000) {
    timer = millis(); // reset the timer
    
    if (newGPSdata)
    {
      digitalWrite(6,!digitalRead(6));
      newGPSdata = false;
      unsigned long age;
      gps.f_get_position(&latitude_gps, &longitude_gps, &age);
      latitude_msg.data = latitude_gps;
      lat_pub.publish( &latitude_msg );
      longitude_msg.data = longitude_gps;
      lon_pub.publish( &longitude_msg );
    }
   }
  
  if (mpu.update()) {
    if (millis() > prev_ms + 100){
    prev_ms = millis();
    send_data_imu();
    digitalWrite(5,!digitalRead(5));
    }
  }
  nh.spinOnce();
}
