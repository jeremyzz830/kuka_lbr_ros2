/* *********************** 
* By Yihao Liu
* Johns Hopkins University
* Updated Oct 25 2021
* 
* MIT licence
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
* IN THE SOFTWARE.
* 
************************** */

#pragma once
#include <UtlFunctions.hpp>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <vector>
#include <geometry_msgs/TransformStamped.h>

// utility functions
namespace UtlFunctions{

	constexpr unsigned int cmd_str_int(const char* str, int h = 0);

	void saveRegistrationData(sensor_msgs::PointCloud a,sensor_msgs::PointCloud b, geometry_msgs::Pose c);

	std::string formated_double_2_string(double a, int dec);

	inline std::string getTimeString();

	std::vector<double> parseString2DoubleVec(std::string s);

	void readCalDataYAMLFile(geometry_msgs::TransformStamped& p, const std::string& filename);
}
