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

#include <vector>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "lbr_kst/srv/register_pnt_cloud.hpp"

namespace UtlCalculations{

	bool registerPntCloud(roskst_msgs::RegisterPntCloud::Request &req, roskst_msgs::RegisterPntCloud::Response &res);

	inline std::vector<std::vector<double>> getZeros3by3();

	inline std::vector<std::vector<double>> getZeros(int n);

	inline std::vector<std::vector<double>> getEye3by3();

	inline std::vector<std::vector<double>> getDiag3by3(std::vector<double> v);

	inline std::vector<std::vector<double>> matrixMult3by3(std::vector<std::vector<double>> X, std::vector<std::vector<double>> Y);

	inline std::vector<std::vector<double>> transpose3by3(std::vector<std::vector<double>> A);

	inline double det3by3(std::vector<std::vector<double>> R);

	std::vector<double> getCentroid(std::vector<std::vector<double>> A);

	std::vector<std::vector<double>> getDeviations(std::vector<std::vector<double>> A, std::vector<double> aCentroid);

	std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>> qrSim3by3(std::vector<std::vector<double>> A);

	std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>, std::vector<std::vector<double>>> svdSim3by3(std::vector<std::vector<double>> A);

	std::vector<double> quat2eul(std::vector<double> q /*x,y,z,w*/);

	geometry_msgs::Quaternion eul2quat(std::vector<double> eul);

};