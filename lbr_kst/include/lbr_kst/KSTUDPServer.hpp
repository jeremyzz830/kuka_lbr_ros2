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
#include <boost/array.hpp>
#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

#include "lbr_kst/action/ptp_line_eef.hpp"

using boost::asio::ip::udp;

class KSTUDPServer
{

public:

	// constructor and start receiving
	KSTUDPServer(rcl::NodeHandle& n, boost::asio::io_context& io_context);

private:

	void start_receive();
	inline void end_server_clean();
	void handle_receive(const boost::system::error_code& error, std::size_t /*bytes_transferred*/);
	inline void cmd_incoming_convert();
	inline void cmd_handle_onofftype();
	void cmd_handle_calcutype();
	inline void cmd_handle_locattype();
	inline void cmd_handle_robottype();
	inline void cmd_wrong_notify();
	void iiwaOffAction();
	void ptpMotion();
	void smtServOn();
	void smtServOff();
	inline geometry_msgs::Point cmd_handle_coord();
	inline geometry_msgs::Point32 cmd_handle_coord32();
	inline geometry_msgs::Quaternion cmd_handle_quat();
	void confirmMsg(std::string msg_cfrm);

	// ros related members
	rcl::Publisher pubTest_;
	rcl::Publisher pubCommand_;
	rcl::Publisher pubSubCommand_;
	rcl::Publisher pubPointCloud_;
	rcl::Publisher pubPosQuatCntct_;
	rcl::Publisher pubPosQuatHdref_;
	rcl::Publisher pubPosQuatCnoff_;
	rcl::Publisher pub_EEFOldpos_;
	actionlib::SimpleActionClient<lbr_kst::PTPLineEEFAction> ac_PTPLineEEF_;
	rcl::ServiceClient cgeteef_;
	rcl::ServiceClient srv_client_stopSmtServo_;
	rcl::ServiceClient srv_client_startSmtServoEEF_;

	// udp related memebrs
	udp::socket socket_;
    udp::socket socket_msgcfrm_;
	udp::endpoint remote_endpoint_;
    udp::endpoint remote_endpoint_msgcfrm_;
	boost::array<char, 55> recv_buffer_; // 55 chars in an encoded command
	
	// encoded command members
	std::string sscmdStr_; // command type 
	std::string sstypStr_; // command sub-type
	std::string sscrdStr_; // coordinate/pose numbers
	int numAllFid_ = 0;
	std_msgs::String msgTest_;
	std_msgs::String msgCommand_;
	std_msgs::String msgSubCommand_;
	geometry_msgs::Pose msgCntct_;
	geometry_msgs::Pose msgHdref_;
	geometry_msgs::Pose msgCnoff_;
	geometry_msgs::Pose tempTrans_;
	sensor_msgs::PointCloud msgPntCld_;
	sensor_msgs::PointCloud digPntCld_;

	// safety flags
	bool flag_headcntct_recvd_ = false;
	bool flag_headheadref_recvd_ = false;
	bool flag_cntctoffset_recvd_ = true;
	bool flag_fid_recvd_ = false;
	std::vector<bool> flag_dig_recvd_;

};