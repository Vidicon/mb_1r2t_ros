#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>

// #include <cstdio>
// #include <iomanip>
#include <sys/ioctl.h>
#include <termios.h>
// #include <asm/termios.h>
// #include <sys/types.h>
// #include <cstdlib>
// #include <fcntl.h>
// #include <sys/time.h>
// #include <unistd.h>
// #include <cmath>
// #include <string>

// #include <stdio.h>
// #include <stdint.h>
// #include <fcntl.h>
// #include <errno.h>
// #include <asm/termios.h>

#if( __linux )
#include <linux/serial.h>
#endif
#define BAUD_RATE B38400
#include <mb_1r2t_ros/mb_1r2t.h>
#define TCGETS2_ 0x802C542A
#define TCSETS2_ 0x402C542B
 // baudrate ioctls


#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

Mb_1r2t::Mb_1r2t(const std::string &port, uint32_t baud_rate):
  port_(port), baud_rate_(baud_rate), serial_(io_, port_)
{
  shutting_down_ = false;

  serial_.set_option(  boost::asio::serial_port::flow_control( boost::asio::serial_port::flow_control::none ) ); 
  serial_.set_option(  boost::asio::serial_port::parity( boost::asio::serial_port::parity::none ) );          
  serial_.set_option( boost::asio::serial_port::stop_bits(  boost::asio::serial_port::stop_bits::one ) );  
  serial_.set_option(  boost::asio::serial_port::character_size( 8 ) );    
  //set_option does not suport non standard baudrates 
  // serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
  
  int  fd = serial_.native_handle();

  int buff[64];
  if((ioctl(fd, TCGETS2_, buff)) < 0)
	{
	printf("BAUD: error to get the serial_struct info:%s\n", strerror(errno));
	}
  buff[2] &= ~CBAUD; 
  buff[2] |= 0x1000; 
  buff[9] = baud_rate_;
  buff[10] = baud_rate_;
  // this works.... thanks pyserial.
  if((ioctl(fd, TCSETS2_, buff)) < 0)
	{
	printf("BAUD: error to set serial_struct:%s\n", strerror(errno));

	}

}

Mb_1r2t::~Mb_1r2t()
{
}

void Mb_1r2t::setCloudCallback(void (*callback)(std::vector<Point_2d> *points))
{
  cloud_callback_ = callback;
}

void Mb_1r2t::setScanCallback(void (*callback)(std::vector<Vector_2d> *vectors))
{
  scan_callback_ = callback;
}


bool Mb_1r2t::scanReady()
{
  return scan_ready_;
}

void Mb_1r2t::parsePacket()
{
  State state = SYNC0;
  Package_header package;
  bool got_package = false;
  boost::array<uint8_t, 160> raw_bytes;

  while (!shutting_down_ && !got_package)
  {

    switch (state)
    {
    case SYNC0:
    {
      boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[0], 1));
      // ROS_INFO("start1 %x", raw_bytes[0]);
      if (raw_bytes[0] == KSYNC0)
      {
        state = SYNC1;
      }
      break;
    }
    case SYNC1:
    {
      boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[1], 1));
      // ROS_INFO("2");
      if (raw_bytes[1] == KSYNC1)
      {
        state = HEADER;
        break;
      }
      state = SYNC0;
      break;
    }
    case HEADER:
    {
      boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[2], 8));
      // ROS_INFO("header");
      package.type = raw_bytes[2];
      package.data_length = raw_bytes[3];
      package.start_angle = raw_bytes[5] << 8 | raw_bytes[4];
      package.stop_angle = raw_bytes[7] << 8 | raw_bytes[6];
      // uint8_t crc_H = raw_bytes[8]
      // uint8_t crc_L = raw_bytes[9];

      state = DATA;
    }
    case DATA:
    {
      uint16_t bytes_to_read = package.data_length * 3;
      boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[10], bytes_to_read));
      // ROS_INFO("type: 0x%02X BIN: "BYTE_TO_BINARY_PATTERN, package.type, BYTE_TO_BINARY(package.type));
      if (package.type & 1)
      {

        if(cloud_callback_ != nullptr)
        {
          cloud_callback_(&points_);
        }
        points_.clear();
        return;
      }

      int16_t diff = package.stop_angle - package.start_angle;
			if(package.stop_angle < package.start_angle)
				diff =  0xB400 - package.start_angle + package.stop_angle;

			int16_t angle_per_sample = 0;
			if(diff > 1)
				angle_per_sample = diff / (package.data_length-1);

      // for (int i = 0; i < package.data_length*3 + 10; i++)
      //   ROS_INFO("[%i]: 0x%02X", i, raw_bytes[i]);

      for (int i = 0; i < package.data_length; i++)
      {
        uint16_t index = 10 + (i * 3) ;
        uint8_t intensity = raw_bytes[index + 0];
        uint8_t distanceL = raw_bytes[index + 1];
        uint8_t distanceH = raw_bytes[index + 2];
        // if(intensity == 1)
        //   continue;

        uint16_t distance = (uint16_t) (distanceH << 8) + (uint16_t)distanceL;
        float distancef = (float)distance / 4000.0;

        float step = M_PI*2;
        float angle = (package.start_angle + angle_per_sample * i);
				float anglef = (step * (angle / 0xB400));

        if(anglef < last_angle_)
        {
          if(scan_callback_ != nullptr)
          {
            scan_callback_(&vectors_);
          }
           vectors_.clear();
        }
        last_angle_ = anglef;
        // ROS_INFO("angle: %f distance:%i,  (%f) intensity %i", anglef, distance, distancef, intensity);
        float angle_inv = (M_PI*2) - anglef;
        Point_2d point;
        point.x = cos(angle_inv) * distancef;
        point.y = sin(angle_inv) * distancef;
        point.intensity = intensity;
        points_.push_back(point);
        
        Vector_2d vector;
        vector.distance = distancef;
        vector.angle = anglef;
        vector.intensity = intensity;
        vectors_.push_back(vector);
      }

      got_package = true;

      state = SYNC0;
      // ROS_INFO("data");
      break;
    }
    default:
    {
      //error
    }
    }
  }
}

float Mb_1r2t::getRotationSpeed()
{
  return rotationSpeed_;
}
