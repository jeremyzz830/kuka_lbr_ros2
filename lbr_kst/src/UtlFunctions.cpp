/* *********************** 
* By Yihao Liu, Joshua Liu
* Johns Hopkins University
* Updated 11/1/2021
* 
* MIT licence
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
* IN THE SOFTWARE.
* 
************************** */

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
void UtlFunctions::saveRegistrationData(sensor_msgs::PointCloud a, sensor_msgs::PointCloud b, geometry_msgs::Pose c)
{
	std::string packpath = ros::package::getPath("roskst");
	std::ofstream filesave(packpath + "/share/data/reg-" + UtlFunctions::getTimeString() + ".txt");
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

std::string UtlFunctions::formated_double_2_string(double a, int dec)
{
	std::stringstream stream;
    stream << std::fixed << std::setprecision(dec) << a;
    std::string s = stream.str();
    return s;
}

inline std::string UtlFunctions::getTimeString()
{
	time_t rawtime;
	struct tm * timeinfo;
	char buffer [80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime (buffer,80,"%Y%m%d-%I%M%p",timeinfo);

	return buffer;
}

std::vector<double> UtlFunctions::parseString2DoubleVec(std::string s)
{
	std::vector<double> vec;
	std::string delimiter = "_";
	size_t pos = 0;
	while ((pos = s.find(delimiter)) != std::string::npos) 
	{
	    vec.push_back(std::stod(s.substr(0, pos)));
	    s.erase(0, pos + delimiter.length());
	}
	return vec;
}

void UtlFunctions::readCalDataYAMLFile(geometry_msgs::TransformStamped& p, const std::string& filename)
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
