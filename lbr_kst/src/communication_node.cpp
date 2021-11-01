/* *********************** 
* By Yihao Liu, Joshua Liu
* Johns Hopkins University
* Updated 10/30/2021
* 
* MIT licence
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
* IN THE SOFTWARE.
* 
************************** */


#include <string>
#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"

#include "lbr_kst/KSTServoing.hpp"
#include "lbr_kst/KSTServoingCmd.hpp"
#include "lbr_kst/UtlCalculations.hpp"


class EEFSaver
{
public:
	EEFSaver(KSTServoingCmd& servoCmd) : cmd_(servoCmd)
	{

	}
	void EEFSaverCallback(const geometry_msgs::TransformStampedConstPtr& msg)
	{
		// if(cmd_.getflg_TMSrealtimectl())
		// {
			std::vector<double> q{
				msg->transform.rotation.x,
				msg->transform.rotation.y,
				msg->transform.rotation.z,
				msg->transform.rotation.w
			};
			std::vector<double> q_ = UtlCalculations::quat2eul(q /*x,y,z,w*/);
			if(!inited_)
			{
				eef.push_back(msg->transform.translation.x); // get calculated realtime eef
				eef.push_back(msg->transform.translation.y);
				eef.push_back(msg->transform.translation.z);

				eef.push_back(q_[0]);
				eef.push_back(q_[1]);
				eef.push_back(q_[2]);

				eef_nominal.push_back(msg->transform.translation.x); // get calculated realtime eef
				eef_nominal.push_back(msg->transform.translation.y);
				eef_nominal.push_back(msg->transform.translation.z);

				eef_nominal.push_back(q_[0]);
				eef_nominal.push_back(q_[1]);
				eef_nominal.push_back(q_[2]);

				ROS_INFO("Nominal pose: %f %f %f %f %f %f", eef_nominal[0], eef_nominal[1], eef_nominal[2], eef_nominal[3], eef_nominal[4], eef_nominal[5]);
				inited_ = true;
			}
			else
			{

				eef[0] = boxlimit(msg->transform.translation.x, eef_nominal[0], 10);
				eef[1] = boxlimit(msg->transform.translation.y, eef_nominal[1], 10);
				eef[2] = boxlimit(msg->transform.translation.z, eef_nominal[2], 10);
				eef[3] = boxlimit(q_[0], eef_nominal[3], 0.1);
				eef[4] = boxlimit(q_[1], eef_nominal[4], 0.1);
				eef[5] = boxlimit(q_[2], eef_nominal[5], 0.1);

				eef[0] = eef_old[0] + gain * (eef[0] - eef_old[0]);
				eef[1] = eef_old[1] + gain * (eef[1] - eef_old[1]);
				eef[2] = eef_old[2] + gain * (eef[2] - eef_old[2]);
				eef[3] = eef_old[3] + gain * (eef[3] - eef_old[3]);
				eef[4] = eef_old[4] + gain * (eef[4] - eef_old[4]);
				eef[5] = eef_old[5] + gain * (eef[5] - eef_old[5]);
			}
		// }
	}
	void EEFOldInitCallback(const geometry_msgs::TransformStampedConstPtr& msg)
	{
		std::vector<double> q{
			msg->transform.rotation.x,
			msg->transform.rotation.y,
			msg->transform.rotation.z,
			msg->transform.rotation.w
		};
		std::vector<double> q_ = UtlCalculations::quat2eul(q /*x,y,z,w*/);
		if(!inited_eefold)
		{
			eef_old.push_back(msg->transform.translation.x);
			eef_old.push_back(msg->transform.translation.y);
			eef_old.push_back(msg->transform.translation.z);
			eef_old.push_back(q_[0]);
			eef_old.push_back(q_[1]);
			eef_old.push_back(q_[2]);
		}
		else
		{
			eef_old[0] = msg->transform.translation.x;
			eef_old[1] = msg->transform.translation.y;
			eef_old[2] = msg->transform.translation.z;
			eef_old[3] = q_[0];
			eef_old[4] = q_[1];
			eef_old[5] = q_[2];
		}
		
	}
	double boxlimit(double a, double nominal, double range)
	{
		double ans;
		if (a > nominal+range)
			ans = nominal+range;
		else if (a < nominal-range)
			ans = nominal-range;
		else
			ans = a;
		return ans;
	}
	std::vector<double> eef;
	std::vector<double> eef_nominal;
	std::vector<double> eef_old;
	double gain = 0.01;

private:
	KSTServoingCmd& cmd_;
	bool inited_ = false;
	bool inited_eefold = false;
	
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin();
  rclcpp::shutdown();
  return 0;
}
