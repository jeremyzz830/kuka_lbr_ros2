#include <UtlROSFunctions.hpp>
#include <UtlFunctions.hpp>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Pose.h>
#include <iomanip>
#include <string>
#include <sstream>
#include <time.h>
#include <ros/ros.h>
#include <fstream>
#include <ros/package.h>
#include <vector>
#include <geometry_msgs/TransformStamped.h>
#include <yaml-cpp/yaml.h>


// utility functions
void UtlROSFunctions::saveRegistrationData(sensor_msgs::PointCloud a, sensor_msgs::PointCloud b, geometry_msgs::Pose c)
{
	std::string packpath = ros::package::getPath("roskst");
	std::ofstream filesave(packpath + "/share/data/reg-" + UtlROSFunctions::getTimeString() + ".txt");
	if(filesave.is_open())
	{
		for(int i=0;i<a.points.size();i++){
			filesave << UtlFunctions::formated_double_2_string(a.points[i].x, 6);
			filesave << ", ";
			filesave << UtlFunctions::formated_double_2_string(a.points[i].y, 6);
			filesave << ", ";	
			filesave << UtlFunctions::formated_double_2_string(a.points[i].z, 6);
			filesave << "\n";
		}
		filesave << "\n";
		for(int i=0;i<a.points.size();i++){
			filesave << UtlFunctions::formated_double_2_string(b.points[i].x, 6);
			filesave << ", ";	
			filesave << UtlFunctions::formated_double_2_string(b.points[i].y, 6);
			filesave << ", ";
			filesave << UtlFunctions::formated_double_2_string(b.points[i].z, 6);
			filesave << "\n";
		}
		filesave << "\n";

		filesave << UtlFunctions::formated_double_2_string(c.position.x, 6);
		filesave << ", ";	
		filesave << UtlFunctions::formated_double_2_string(c.position.y, 6);
		filesave << ", ";
		filesave << UtlFunctions::formated_double_2_string(c.position.z, 6);
		filesave << "\n";
		filesave << "quat: w,x,y,z\n";
		filesave << UtlFunctions::formated_double_2_string(c.orientation.w, 6);
		filesave << ", ";
		filesave << UtlFunctions::formated_double_2_string(c.orientation.x, 6);
		filesave << ", ";	
		filesave << UtlFunctions::formated_double_2_string(c.orientation.y, 6);
		filesave << ", ";
		filesave << UtlFunctions::formated_double_2_string(c.orientation.z, 6);
		filesave << "\n";

		filesave.close();
	}
	else 
		ROS_INFO("Unable to open file");
}

inline std::string UtlROSFunctions::getTimeString()
{
	time_t rawtime;
	struct tm * timeinfo;
	char buffer [80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime (buffer,80,"%Y%m%d-%I%M%p",timeinfo);

	return buffer;
}

void UtlROSFunctions::readCalDataYAMLFile(geometry_msgs::TransformStamped& p, const std::string& filename)
{

	YAML::Node f = YAML::LoadFile(filename);

	p.transform.translation.x = f["tx"].as<float>();
	p.transform.translation.y = f["ty"].as<float>();
	p.transform.translation.z = f["tz"].as<float>();

	p.transform.rotation.x = f["rx"].as<float>();
	p.transform.rotation.y = f["ry"].as<float>();
	p.transform.rotation.z = f["rz"].as<float>();
	p.transform.rotation.w = f["rw"].as<float>();

}