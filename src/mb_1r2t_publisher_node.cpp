#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <mb_1r2t_ros/mb_1r2t.h>
#include <vector>

#define BAUDRATE 153600

ros::Publisher g_cloud_pub;
ros::Publisher g_hz_pub;
ros::Publisher g_scan_pub;

sensor_msgs::PointCloud2 g_init_cloud_msg;
sensor_msgs::LaserScan g_init_scan_msg;




typedef struct __attribute__((packed)){
  float x;
  float y;
  float z;
  uint8_t   intensity;
} Pointdata;

typedef union 
{
  Pointdata point_data;
  uint8_t bytes[sizeof(Pointdata)];
}Point_u;

sensor_msgs::PointCloud2 createPointcloudMsg(std::string frame_id)
{
  sensor_msgs::PointCloud2 cloud_msg;
  cloud_msg.header.frame_id = frame_id;
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.is_bigendian = false;
  cloud_msg.is_dense = false; // there are invalid points (inf or nan)
  cloud_msg.height = 1; 
  // cloud_msg.width = points->size();
  cloud_msg.point_step = 13; // Float32 = 4 byte * 3(xyz) + 1 byte intensity = 13 bytes

  sensor_msgs::PointField field;
  field.name = "x";
  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.count = 1;
  field.offset = 0;
  cloud_msg.fields.push_back(field);
  field.name = "y";
  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.count = 1;
  field.offset = 4;
  cloud_msg.fields.push_back(field);
  field.name = "z";
  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.count = 1;
  field.offset = 8;
  cloud_msg.fields.push_back(field);
  field.name = "intensity";
  field.datatype = sensor_msgs::PointField::UINT8;
  field.count = 1;
  field.offset = 12;
  cloud_msg.fields.push_back(field);
  return cloud_msg;
}

sensor_msgs::LaserScan createScanMsg(std::string frame_id)
{
  sensor_msgs::LaserScan scan;
  scan.header.frame_id = frame_id;
  scan.range_min = 0.0;
  scan.angle_max = 2.0*M_PI;
  scan.range_min = 0.11;
  scan.range_max = 8.0;
  // scan.angle_increment = (2*M_PI)/400;
  // scan.ranges.resize(400);
  // scan.intensities.resize(400);
  return scan;
}

void cloudCallback(std::vector<Point_2d> *points)
{
  sensor_msgs::PointCloud2 cloud_msg = g_init_cloud_msg;
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.width = points->size();

  
  for(int i = 0; i < points->size(); i++) 
  { 
    Point_u data;
    data.point_data.x = points->at(i).x;
    data.point_data.y = points->at(i).y;
    data.point_data.z = 0.0;
    data.point_data.intensity = points->at(i).intensity;

    for (size_t i2 = 0; i2 < 13; i2++)
    {
      cloud_msg.data.push_back(data.bytes[i2]);
    }
  }
  
  
  // ROS_INFO("##### callback ##### size: %lu",points->size());
  g_cloud_pub.publish(cloud_msg);
}

void scanCallback(std::vector<Vector_2d> *vectors)
{
  if(vectors->size() == 0)
  {
    return;
  }

  sensor_msgs::LaserScan scan_msg = g_init_scan_msg;
  scan_msg.header.stamp = ros::Time::now();
  scan_msg.angle_increment = 2*M_PI / vectors->size();
  scan_msg.time_increment = 0.2 / vectors->size(); // TODO: not corectly calculated yet, but better than nothing...
  scan_msg.scan_time = 0.2;//TODO: not the real value
  for(int i = 0; i < vectors->size(); i++) 
  { 
    Vector_2d vector = vectors->at((vectors->size()-1) - i);
    scan_msg.ranges.push_back(vector.distance);
    scan_msg.intensities.push_back(vector.intensity);
  }
  
  
  ROS_INFO("##### callback ##### size: %lu",vectors->size());
  g_scan_pub.publish(scan_msg);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mb_1r2t_publisher");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");

  std::string port;
  
  std::string frame_id;

  priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
  priv_nh.param("frame_id", frame_id, std::string("laser"));

  
  
  g_scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 10);
  g_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("cloud", 10);
  g_hz_pub = n.advertise<std_msgs::Float32>("hz",10);

  ROS_INFO("Starting mb_1r2t_publisher (%s@%i)",port.c_str(),BAUDRATE);

  // dont need to redo this every time.
  g_init_cloud_msg = createPointcloudMsg(frame_id);
  g_init_scan_msg = createScanMsg(frame_id);
 
  
  Mb_1r2t lidar(port, BAUDRATE);
  lidar.setScanCallback(&scanCallback);
  lidar.setCloudCallback(&cloudCallback);

  while (ros::ok())
  {

    lidar.parsePacket();

    // for(int i = 0; i < 400; i++)
    // {
    //   scan.ranges[i] =  lidar.distanceArray[i] / 1000.0;
    //   scan.intensities[i] =  lidar.qualityArray[i];
    // }
    // scan.time_increment = 1.0 / lidar.getRotationSpeed() / 400;
    // scan.header.stamp = ros::Time::now();
    // scan_pub.publish(scan);

    // rpmsMsg.data = lidar.getRotationSpeed();
    // g_hz_pub.publish(rpmsMsg);
  }
  lidar.close();

  return 0;
}
