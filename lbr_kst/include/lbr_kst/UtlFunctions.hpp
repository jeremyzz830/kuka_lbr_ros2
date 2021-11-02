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

// utility functions
namespace UtlFunctions{

	constexpr unsigned int cmd_str_int(const char* str, int h = 0);

	void saveRegistrationData(sensor_msgs::PointCloud a,sensor_msgs::PointCloud b, geometry_msgs::Pose c);

	std::string formated_double_2_string(double a, int dec);

	inline std::string getTimeString();

	std::vector<double> parseString2DoubleVec(std::string s);

	void readCalDataYAMLFile(geometry_msgs::TransformStamped& p, const std::string& filename);
}
