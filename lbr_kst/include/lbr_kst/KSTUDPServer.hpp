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

#include <actionlib/client/simple_action_client.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <roskst_msgs/PTPLineEEFAction.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud.h>

#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

class KSTUDPServer
{

public:

	// constructor and start receiving
	KSTUDPServer(ros::NodeHandle& n, boost::asio::io_context& io_context);

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
	ros::NodeHandle& n_;
	ros::Publisher pubTest_;
	ros::Publisher pubCommand_;
	ros::Publisher pubSubCommand_;
	ros::Publisher pubPointCloud_;
	ros::Publisher pubPosQuatCntct_;
	ros::Publisher pubPosQuatHdref_;
	ros::Publisher pubPosQuatCnoff_;
	ros::Publisher pub_EEFOldpos_;
	actionlib::SimpleActionClient<roskst_msgs::PTPLineEEFAction> ac_PTPLineEEF_;
	ros::ServiceClient cgeteef_;
	ros::ServiceClient srv_client_stopSmtServo_;
	ros::ServiceClient srv_client_startSmtServoEEF_;

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