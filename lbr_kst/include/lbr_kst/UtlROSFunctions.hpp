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

#pragma once

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "geometry_msgs/Quaternion.h"

// utility functions, using ROS stuff
namespace UtlROSFunctions{

	void saveRegistrationData(sensor_msgs::PointCloud a,sensor_msgs::PointCloud b, geometry_msgs::Pose c);

	inline std::string getTimeString();

	void readCalDataYAMLFile(geometry_msgs::TransformStamped& p, const std::string& filename);

	std::vector<double> quat2eul(std::vector<double> q /*x,y,z,w*/);

	geometry_msgs::Quaternion eul2quat(std::vector<double> eul);
}
