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

#include <KSTUDPServer.hpp>
#include <UtlCalculations.hpp>

#include <yaml-cpp/yaml.h>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <string>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>

#include <roskst_msgs/PointCloudPair.h>
#include <roskst_msgs/RegisterPntCloud.h>
#include <roskst_msgs/GetEEF.h>
#include <roskst_msgs/PTPLineEEFAction.h>
#include <roskst_msgs/NetiiwaCloseAction.h>
#include <roskst_msgs/SmtSrvoStop.h>
#include <roskst_msgs/SmtSrvoStartEEF.h>
#include <UtlFunctions.hpp>

using boost::asio::ip::udp;

// somehow this constexpr function cannot be put into the UtlFunctions.cpp implementation file.
// Error: used before definition. So I had to put it here. Suspect CMake compilation order problem.
constexpr unsigned int UtlFunctions::cmd_str_int(const char* str, int h /* = 0 */)
{ 	
    // https://stackoverflow.com/questions/16388510/evaluate-a-string-with-a-switch-in-c/16388594
    return !str[h] ? 5381 : (cmd_str_int(str, h+1) * 33) ^ str[h];
}

// constructor and start receiving
KSTUDPServer::KSTUDPServer(ros::NodeHandle& n, boost::asio::io_context& io_context)	: 
	n_(n), 
	ac_PTPLineEEF_("/TMSKuka/movePTPLineEEF", true),
	socket_(io_context, udp::endpoint(udp::v4(), 8051)),
    socket_msgcfrm_(io_context)
{
	cgeteef_ = n_.serviceClient<roskst_msgs::GetEEF>("/TMSKuka/GetEEF");
	srv_client_stopSmtServo_ = n_.serviceClient<roskst_msgs::SmtSrvoStop>("/TMSKuka/SmtSrvoStop");
	srv_client_startSmtServoEEF_ = n_.serviceClient<roskst_msgs::SmtSrvoStartEEF>("/TMSKuka/SmtSrvoStartEEF");
	socket_msgcfrm_.open(boost::asio::ip::udp::v4());
	remote_endpoint_msgcfrm_ = udp::endpoint(udp::v4(), 8295);
    // remote_endpoint_msgcfrm_ = udp::endpoint(boost::asio::ip::address_v4::from_string(""), 8295);
	pubTest_ = n_.advertise<std_msgs::String>("/TMSKuka/PubTest", 50);
	pubCommand_ = n_.advertise<std_msgs::String>("/TMSKuka/PubCommand", 50);
	pubSubCommand_ = n_.advertise<std_msgs::String>("/TMSKuka/PubSubCommand", 50);
	pubPointCloud_ = n_.advertise<roskst_msgs::PointCloudPair>("/TMSKuka/PubPointCloud", 10);
	pubPosQuatCntct_ = n_.advertise<geometry_msgs::Pose>("/TMSKuka/PubPosQuatCntct", 10);
	pubPosQuatHdref_ = n_.advertise<geometry_msgs::Pose>("/TMSKuka/PubPosQuatHdref", 10);
	pubPosQuatCnoff_ = n_.advertise<geometry_msgs::Pose>("/TMSKuka/PubPosQuatCnoff", 10);
	pub_EEFOldpos_ = n_.advertise<geometry_msgs::TransformStamped>("/TMSKuka/EEFOldposLatch", 10);

	// Get the default P_contact_offset
	std::string packpath = ros::package::getPath("roskst");
	geometry_msgs::TransformStamped p_cntct_offset;
	UtlFunctions::readCalDataYAMLFile(p_cntct_offset, packpath + "/share/data/p_cntct_offset.yaml");
	msgCnoff_.position.x = p_cntct_offset.transform.translation.x;
	msgCnoff_.position.y = p_cntct_offset.transform.translation.y;
	msgCnoff_.position.z = p_cntct_offset.transform.translation.z;
	msgCnoff_.orientation = p_cntct_offset.transform.rotation;

	KSTUDPServer::start_receive();
}

void KSTUDPServer::start_receive()
{
	ROS_INFO("Wait msg");
	socket_.async_receive_from(
		boost::asio::buffer(recv_buffer_), 
		remote_endpoint_,
		boost::bind(
			&KSTUDPServer::handle_receive, this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred
		)
	);
}

inline void KSTUDPServer::end_server_clean()
{
	KSTUDPServer::iiwaOffAction();
	socket_msgcfrm_.close();
}

void KSTUDPServer::handle_receive(const boost::system::error_code& error,
		std::size_t /*bytes_transferred*/)
{
	if (!error)
	{	
		ROS_INFO("Received msg");
		// string handling
		
		KSTUDPServer::cmd_incoming_convert();

		msgTest_.data = sscmdStr_;
		pubTest_.publish(msgTest_);
		ros::spinOnce();

		// ROS_INFO("switch block start");
		switch (UtlFunctions::cmd_str_int(sscmdStr_.c_str()))
		{
			case UtlFunctions::cmd_str_int("onoff") : // on and off commands. mainly for connections
			{
				KSTUDPServer::cmd_handle_onofftype();
				ROS_INFO("On off type command executed");
				break;
			}
			case UtlFunctions::cmd_str_int("calcu") : // calculation commands. need to execute some calculations
			{
				KSTUDPServer::cmd_handle_calcutype();
				ROS_INFO("Calculation type command executed");
				break;
			}
			case UtlFunctions::cmd_str_int("locat") : // location commands. need to receive and/or send transformations
			{
				KSTUDPServer::cmd_handle_locattype();
				ROS_INFO("Coordinate type command executed");
				break;
			}
			case UtlFunctions::cmd_str_int("robot") : // robot motion commands. need to move robot
			{
				KSTUDPServer::cmd_handle_robottype();
				ROS_INFO("Robot motion type command executed");
				break;
			}
			default : // wrong commands
			{
				KSTUDPServer::cmd_wrong_notify();
				ROS_INFO("Commant not recgonized");
				break;
			}
		}
		// ROS_INFO("switch block end");


		KSTUDPServer::start_receive();
	}
}

inline void KSTUDPServer::cmd_incoming_convert()
{
	std::stringstream sscmd, sstyp;
	for(int i=0;i<5;i++) // 55 chars in an encoded command
		sscmd << recv_buffer_[i];
	for(int i=6;i<11;i++)
		sstyp << recv_buffer_[i];
	sscmdStr_ = sscmd.str();
	sstypStr_ = sstyp.str();
}

inline void KSTUDPServer::cmd_handle_onofftype()
{
	switch (UtlFunctions::cmd_str_int(sstypStr_.c_str()))
	{
		case UtlFunctions::cmd_str_int("gjpos"): // get robot joints
		{
			ROS_INFO("User asks for jpos");

			break;
		}
		case UtlFunctions::cmd_str_int("gepos"): // get robot eef
		{
			ROS_INFO("User asks for eefpos");
			break;
		}
		case UtlFunctions::cmd_str_int("rosff") : 
		{
			ROS_INFO("User interrupted from GUI");
			KSTUDPServer::end_server_clean();
			throw(std::runtime_error("error"));
			break;
		}
		case UtlFunctions::cmd_str_int("iiwon") : 
		{
            msgCommand_.data = "onoff_iiwon";
            pubCommand_.publish(msgCommand_);
            ros::spinOnce();
            ROS_INFO("User starts iiwa");
			break;
		}
		case UtlFunctions::cmd_str_int("iiwff") : 
		{

            msgCommand_.data = "onoff_iiwff";
            pubCommand_.publish(msgCommand_);
            ros::spinOnce();

            KSTUDPServer::iiwaOffAction();
			break;
		}
		case UtlFunctions::cmd_str_int("polon") : 
		{
            // msgCommand_.data = "onoff_polon";
            // pubCommand_.publish(msgCommand_);
            // ros::spinOnce();
            // ROS_INFO("User starts polaris");
			break;
		}
		case UtlFunctions::cmd_str_int("polff") : 
		{
            // msgCommand_.data = "onoff_polff";
            // pubCommand_.publish(msgCommand_);
            // ros::spinOnce();
            // ROS_INFO("User shuts down polaris");
			break;
		}
        case UtlFunctions::cmd_str_int("matff") : 
		{
            msgCommand_.data = "onoff_matff";
            pubCommand_.publish(msgCommand_);
            ros::spinOnce();
            ROS_INFO("User shuts down Matlab server");
			break;
		}
		case UtlFunctions::cmd_str_int("digat") : 
		{
            msgCommand_.data = "onoff_digat";
            pubCommand_.publish(msgCommand_);
            ros::spinOnce();
            ROS_INFO("User starts automatic digiztization");

            if(flag_fid_recvd_)
            {
            	ros::Publisher pub_beep = n_.advertise<std_msgs::Int32>("/NDI/beep", 10);
            	ros::Duration(3).sleep();
            	std_msgs::Int32 beep_num;
            	beep_num.data = 3;
            	pub_beep.publish(beep_num); ros::spinOnce();
            	ros::Duration(7).sleep();
            	for(int i=0;i<digPntCld_.points.size();i++)
            	{
            		beep_num.data = 2;
            		ros::Duration(7).sleep();
            		pub_beep.publish(beep_num); ros::spinOnce();
            		geometry_msgs::TransformStampedConstPtr curdigPtr = ros::topic::waitForMessage<geometry_msgs::TransformStamped>("/TMSKuka/PtrtipWRTHeadRef");
					digPntCld_.points[i].x = curdigPtr->transform.translation.x;
					digPntCld_.points[i].y = curdigPtr->transform.translation.y;
					digPntCld_.points[i].z = curdigPtr->transform.translation.z;
					flag_dig_recvd_[i] = true;
					ROS_INFO("User digitized one point (#%d)", i);
            	}
            }
            else
            {
            	ROS_INFO("User digitized failed! Fiducials not received yet!");
            }
			break;
		}
		case UtlFunctions::cmd_str_int("odreg") : 
		{	
			std::string packpath = ros::package::getPath("roskst");
			YAML::Node f = YAML::LoadFile(packpath + "/share/config/default.yaml");
			msgHdref_.position.x = f["regtx"].as<float>();
			msgHdref_.position.y = f["regty"].as<float>();
			msgHdref_.position.z = f["regtz"].as<float>();
			msgHdref_.orientation.x = f["regrx"].as<float>();
			msgHdref_.orientation.y = f["regry"].as<float>();
			msgHdref_.orientation.z = f["regrz"].as<float>();
			msgHdref_.orientation.w = f["regrw"].as<float>();
        	pubPosQuatHdref_.publish(msgHdref_); ros::spinOnce();
        	ROS_INFO("Used a previous registration successful!");
        	flag_headheadref_recvd_ = true;
			break;
		}
		default : // wrong commands
		{
            if(sstypStr_.substr(0,3) == "dig") 
            {
                msgCommand_.data = "onoff_dig";
                pubCommand_.publish(msgCommand_); ros::spinOnce();
                msgSubCommand_.data = sstypStr_.substr(3,2);
                pubSubCommand_.publish(msgSubCommand_); ros::spinOnce();

                if(flag_fid_recvd_)
                {
                	geometry_msgs::TransformStampedConstPtr curdigPtr = ros::topic::waitForMessage<geometry_msgs::TransformStamped>("/TMSKuka/PtrtipWRTHeadRef");
                	int i = std::stoi(sstypStr_.substr(3,2));
					digPntCld_.points[i].x = curdigPtr->transform.translation.x;
					digPntCld_.points[i].y = curdigPtr->transform.translation.y;
					digPntCld_.points[i].z = curdigPtr->transform.translation.z;
					flag_dig_recvd_[i] = true;
					ROS_INFO("User digitized one point (#%s)", sstypStr_.substr(3,2).c_str());
					KSTUDPServer::confirmMsg("msg recvd");
                }
                else
                {
                	ROS_INFO("User digitized one point (#%s) failed! Fiducials not received yet!", sstypStr_.substr(3,2).c_str());
                }
            }
            else if(sstypStr_.substr(0,3) == "del")
            {
                msgCommand_.data = "onoff_del";
                pubCommand_.publish(msgCommand_); ros::spinOnce();
                msgSubCommand_.data = sstypStr_.substr(3,2);
                pubSubCommand_.publish(msgSubCommand_); ros::spinOnce();

                int i = std::stoi(sstypStr_.substr(3,2));
                if(flag_dig_recvd_[i])
                {
                	// flag_dig_recvd.erase(flag_dig_recvd.begin() + i);
                	// digPntCld_.points.erase(digPntCld_.points.begin() + i); // need these two when delete a collected fiducial
                	ROS_INFO("User deleted digitized point (#%s)", sstypStr_.substr(3,2).c_str());
                }
                else
                {
                	ROS_INFO("User deleted digitized point (#%s) failed! Fiducial not digitized!", sstypStr_.substr(3,2).c_str());
                }
            }
            else
            {
                KSTUDPServer::cmd_wrong_notify();
            }
            break;
		}
	}	
}

void KSTUDPServer::cmd_handle_calcutype()
{
	switch (UtlFunctions::cmd_str_int(sstypStr_.c_str()))
	{
		case UtlFunctions::cmd_str_int("fidrg"):
		{
			msgCommand_.data = "calcu_fidrg";
            pubCommand_.publish(msgCommand_); ros::spinOnce();
            ROS_INFO("Register!");

            roskst_msgs::PointCloudPair pcp;
            pcp.fidCloud = msgPntCld_.points;
            pcp.digCloud = digPntCld_.points;

            ros::ServiceClient srv_client_rpc = n_.serviceClient<roskst_msgs::RegisterPntCloud>("/TMSKuka/RegisterPntCloud");
            roskst_msgs::RegisterPntCloud rpc;
            rpc.request.pcp = pcp;
            if(srv_client_rpc.call(rpc))
            {
            	msgHdref_ = rpc.response.res;
            	pubPosQuatHdref_.publish(msgHdref_); ros::spinOnce();
            	ROS_INFO("Register successful!");
            	UtlFunctions::saveRegistrationData(digPntCld_, msgPntCld_, msgHdref_);
            	ROS_INFO("Registration data saved.");
            }
            else
            {
            	ROS_INFO("Register failed!");
            }
            flag_headheadref_recvd_ = true;
			break;
		}
		default : 
		{
			KSTUDPServer::cmd_wrong_notify();	
			break;
		}
	}
}

inline void KSTUDPServer::cmd_handle_locattype()
{
	switch (UtlFunctions::cmd_str_int(sstypStr_.c_str()))
	{
		case UtlFunctions::cmd_str_int("ctctr"):
		{
			msgCntct_.orientation = KSTUDPServer::cmd_handle_quat();
			KSTUDPServer::confirmMsg("msg recvd");
			break;
		}
		case UtlFunctions::cmd_str_int("ctctt"):
		{
            // Order has to be: first ctctr then ctctt, 
            // because the server handles it in this way 
            msgCommand_.data = "locat_ctct";
            pubCommand_.publish(msgCommand_); ros::spinOnce();
			msgCntct_.position = KSTUDPServer::cmd_handle_coord();
            pubPosQuatCntct_.publish(msgCntct_); ros::spinOnce();
            flag_headcntct_recvd_ = true;
			break;
		}
		default : // wrong commands
		{
            if (sstypStr_[0] == 'f')
			{
				try
				{
					int numAllFid = std::stoi(sstypStr_.substr(1,2));
					int numFid = std::stoi(sstypStr_.substr(3,2));

					KSTUDPServer::confirmMsg("msg recvd");
                    
					if(numFid!=-1)
					{
						msgPntCld_.points.push_back(cmd_handle_coord32());
						ROS_INFO("%s", std::to_string(numFid).c_str());
						ROS_INFO("%s", std::to_string(numAllFid-1).c_str());
						if(numFid==(numAllFid-1))
						{
							msgCommand_.data = "locat_f";
		                    pubCommand_.publish(msgCommand_); ros::spinOnce();
		                    msgSubCommand_.data = std::to_string(numAllFid);
		                    pubSubCommand_.publish(msgSubCommand_); ros::spinOnce();
		                    ROS_INFO("Fiducial sent");

		                    flag_fid_recvd_ = true;
						}
					}
					else
					{
						msgPntCld_.points.clear();
						numAllFid_ = numAllFid;
						for(int i=0;i<numAllFid_;i++)
						{
							geometry_msgs::Point32 tempDigPhold;
							digPntCld_.points.push_back(tempDigPhold);
							flag_dig_recvd_.push_back(false);
						}
						ROS_INFO("Fiducial array initialized");
					}
				}
				catch(std::exception& e)
				{
					ROS_INFO("Fiducial array handling problem");
					KSTUDPServer::end_server_clean();
					throw;
				}
			}
            else{
                KSTUDPServer::cmd_wrong_notify();
                break;
            }
		}
	}
}

inline void KSTUDPServer::cmd_handle_robottype()
{
	switch (UtlFunctions::cmd_str_int(sstypStr_.c_str()))
	{
		case UtlFunctions::cmd_str_int("ptpmo"):
		{
			KSTUDPServer::ptpMotion();
			KSTUDPServer::confirmMsg("complete");
			break;
		}
		case UtlFunctions::cmd_str_int("ptpcl"):
		{
			msgCnoff_.position.x = 0.0;
			msgCnoff_.position.y = 0.0;
			msgCnoff_.position.z = 5.0;
			pubPosQuatCnoff_.publish(msgCnoff_); ros::spinOnce();
			KSTUDPServer::ptpMotion();
			KSTUDPServer::confirmMsg("complete");
			break;
		}
		case UtlFunctions::cmd_str_int("clofr"):
		{
			tempTrans_.orientation = KSTUDPServer::cmd_handle_quat();
			tf2::Quaternion temp_quat1, temp_quat2;
			tf2::convert(tempTrans_.orientation , temp_quat1);
			tf2::convert(msgCnoff_.orientation , temp_quat2);
			tf2::convert(temp_quat1 * temp_quat2, msgCnoff_.orientation);
			KSTUDPServer::confirmMsg("msg recvd");
			break;
		}
		case UtlFunctions::cmd_str_int("cloft"):
		{
			tempTrans_.position = KSTUDPServer::cmd_handle_coord();
			msgCnoff_.position.x += tempTrans_.position.x;
			msgCnoff_.position.y += tempTrans_.position.y;
			msgCnoff_.position.z += tempTrans_.position.z;
			pubPosQuatCnoff_.publish(msgCnoff_); ros::spinOnce();
			KSTUDPServer::ptpMotion();
			KSTUDPServer::confirmMsg("complete");
			break;
		}
		case UtlFunctions::cmd_str_int("smton"): // start smart servo
		{
			if(flag_headcntct_recvd_ && flag_headheadref_recvd_ && flag_cntctoffset_recvd_){
				KSTUDPServer::smtServOn();
				KSTUDPServer::confirmMsg("started");
			}
			break;
		}
		case UtlFunctions::cmd_str_int("smtof"): // stop smart servo
		{
			KSTUDPServer::smtServOff();
			KSTUDPServer::confirmMsg("stopped");
			break;
		}
		default : // wrong commands
		{
			KSTUDPServer::cmd_wrong_notify();
			break;
		}
	}
}

inline void KSTUDPServer::cmd_wrong_notify() // send to slicer: non-defined command
{
    ROS_INFO("Command not recgonized.");
}

void KSTUDPServer::iiwaOffAction()
{
	actionlib::SimpleActionClient<roskst_msgs::NetiiwaCloseAction> ac("/TMSKuka/rqstNetiiwaClose", true);
	roskst_msgs::NetiiwaCloseGoal gl;
	gl.cl = true;
	ac.waitForServer(); // have to have this so that action client and server connects
	ac.sendGoal(gl);
	ROS_INFO("Action sent.");
	
	bool finished_before_timeout = ac.waitForResult(ros::Duration(3.0)); 
	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
		ROS_INFO("Action did not finish before the time out.");
    ROS_INFO("User shuts down iiwa");
}

void KSTUDPServer::ptpMotion()
{
	roskst_msgs::PTPLineEEFGoal gl;
	roskst_msgs::GetEEF sgeteef;
	std::string packpath = ros::package::getPath("roskst");
	YAML::Node f = YAML::LoadFile(packpath + "/share/config/default.yaml");
 	if(flag_headcntct_recvd_ && flag_headheadref_recvd_ && flag_cntctoffset_recvd_)
 	{
 		
		int whichRef = f["relyOnWhichRef"].as<int>();
 		if(whichRef==0)
 		{
 			sgeteef.request.a = 1;
 			cgeteef_.call(sgeteef);
 			pub_EEFOldpos_.publish(sgeteef.response.eef);	ros::spinOnce();
 		}
 		try
 		{
	 		geometry_msgs::TransformStampedConstPtr p_base_EEF_ptr = 
	 			ros::topic::waitForMessage<geometry_msgs::TransformStamped>("/TMSKuka/CalculatedEEF", ros::Duration(5.0));
	 		gl.eef.x = p_base_EEF_ptr->transform.translation.x;
	 		gl.eef.y = p_base_EEF_ptr->transform.translation.y;
	 		gl.eef.z = p_base_EEF_ptr->transform.translation.z;
	 		std::vector<double> rotquat{
	 			p_base_EEF_ptr->transform.rotation.x,
	 			p_base_EEF_ptr->transform.rotation.y,
	 			p_base_EEF_ptr->transform.rotation.z,
	 			p_base_EEF_ptr->transform.rotation.w
	 		};
	 		std::vector<double> eulang = UtlCalculations::quat2eul(rotquat /*x,y,z,w*/);
	 		gl.eef.rz = eulang[0];
	 		gl.eef.ry = eulang[1];
	 		gl.eef.rx = eulang[2];
	 		ac_PTPLineEEF_.waitForServer();
			ac_PTPLineEEF_.sendGoal(gl);

			ROS_INFO("Action sent.");
		
			// bool finished_before_timeout = ac_PTPLineEEF_.waitForResult(ros::Duration(15.0)); 
			// if (finished_before_timeout)
			// {
			// 	actionlib::SimpleClientGoalState state = ac_PTPLineEEF_.getState();
			// 	ROS_INFO("Action finished: %s",state.toString().c_str());
			// }
			// else
			// 	ROS_INFO("Action did not finish before the time out.");
		}
		catch(...)
		{
			ROS_INFO("Could not calculate the EEF. Check tf frames.");
		}
 		
 	}   
}

inline geometry_msgs::Point KSTUDPServer::cmd_handle_coord()
{
	std::stringstream ssx, ssy, ssz;
	for(int i=12;i<22;i++){
		ssx << recv_buffer_[i];
		ssy << recv_buffer_[i+11];
		ssz << recv_buffer_[i+22];
	}

	geometry_msgs::Point curFid;
	curFid.x = std::stod(ssx.str());
	curFid.y = std::stod(ssy.str());
	curFid.z = std::stod(ssz.str());

	return curFid;
}

inline geometry_msgs::Point32 KSTUDPServer::cmd_handle_coord32()
{
	std::stringstream ssx, ssy, ssz;
	for(int i=12;i<22;i++){
		ssx << recv_buffer_[i];
		ssy << recv_buffer_[i+11];
		ssz << recv_buffer_[i+22];
	}

	geometry_msgs::Point32 curFid;
	curFid.x = std::stod(ssx.str());
	curFid.y = std::stod(ssy.str());
	curFid.z = std::stod(ssz.str());

	return curFid;
}

inline geometry_msgs::Quaternion KSTUDPServer::cmd_handle_quat()
{
	std::stringstream ssx, ssy, ssz, ssw;
	for(int i=12;i<22;i++){
		ssx << recv_buffer_[i];
		ssy << recv_buffer_[i+11];
		ssz << recv_buffer_[i+22];
		ssw << recv_buffer_[i+33];
	}

	geometry_msgs::Quaternion curFid;
	curFid.x = std::stod(ssx.str());
	curFid.y = std::stod(ssy.str());
	curFid.z = std::stod(ssz.str());
	curFid.w = std::stod(ssw.str());

	return curFid;
}

void KSTUDPServer::confirmMsg(std::string msg_cfrm)
{
    boost::system::error_code ignored_error;
    socket_msgcfrm_.send_to(boost::asio::buffer(msg_cfrm),
        remote_endpoint_msgcfrm_, 0, ignored_error);
}

void KSTUDPServer::smtServOn()
{
	roskst_msgs::SmtSrvoStartEEF srv;
	srv.request.a = true;
	if(srv_client_startSmtServoEEF_.call(srv))
	{
		ROS_INFO("realtime smart servo EEF started");
	}
}

void KSTUDPServer::smtServOff()
{
	roskst_msgs::SmtSrvoStop srv;
	srv.request.a = true;
	if(srv_client_stopSmtServo_.call(srv))
	{
		ROS_INFO("realtime smart servo EEF stopped");
	}
}