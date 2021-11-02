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

#include <ros/ros.h>
#include <roskst_msgs/RegisterPntCloud.h>
#include <vector>
#include <limits>
#define _USE_MATH_DEFINES
#include <math.h> 
#include <cmath>
#include <tuple>
#include <UtlCalculations.hpp>
#include <geometry_msgs/Quaternion.h>


inline std::vector<std::vector<double>> UtlCalculations::getZeros3by3()
{
	std::vector<double> vec1{0.0, 0.0, 0.0};
	std::vector<double> vec2{0.0, 0.0, 0.0};
	std::vector<double> vec3{0.0, 0.0, 0.0};
	std::vector<std::vector<double>> H{vec1, vec2, vec3};
	return H;
}

inline std::vector<std::vector<double>> UtlCalculations::getZeros(int n)
{
	std::vector<std::vector<double>> I;
    for(int i = 0; i < n; i++)
    {
    	std::vector<double> vec{ 0.0f, 0.0f, 0.0f };
        I.push_back(vec);
    }
    return I;
}

inline std::vector<std::vector<double>> UtlCalculations::getEye3by3()
{
	std::vector<double> vec1{1.0, 0.0, 0.0};
	std::vector<double> vec2{0.0, 1.0, 0.0};
	std::vector<double> vec3{0.0, 0.0, 1.0};
	std::vector<std::vector<double>> H{vec1, vec2, vec3};
	return H;
}

inline std::vector<std::vector<double>> UtlCalculations::getDiag3by3(std::vector<double> v)
{
	std::vector<std::vector<double>> D = UtlCalculations::getZeros3by3();
	D[0][0] = v[0];
    D[1][1] = v[1];
    D[2][2] = v[2];
    return D;
}

inline std::vector<std::vector<double>> UtlCalculations::matrixMult3by3(std::vector<std::vector<double>> X, std::vector<std::vector<double>> Y)
{
	std::vector<std::vector<double>> A = UtlCalculations::getZeros3by3();
	for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                A[i][j] += X[i][k] * Y[k][j];
    return A;
}

inline std::vector<std::vector<double>> UtlCalculations::transpose3by3(std::vector<std::vector<double>> A)
{
	std::vector<std::vector<double>> H = UtlCalculations::getZeros3by3();
	for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            H[j][i] = A[i][j];
    return H;
}

inline double UtlCalculations::det3by3(std::vector<std::vector<double>> R)
{
	return R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]) - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0]) + R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1]);
}

std::vector<double> UtlCalculations::getCentroid(std::vector<std::vector<double>> A)
{
	int n = A.size();
	std::vector<double> aCentroid{0.0,0.0,0.0};
	for (int i = 0; i < n; i++)
    {
        aCentroid[0] += A[i][0];
        aCentroid[1] += A[i][1];
        aCentroid[2] += A[i][2];
    }
    aCentroid[0] = (1.0 / n) * aCentroid[0];
    aCentroid[1] = (1.0 / n) * aCentroid[1];
    aCentroid[2] = (1.0 / n) * aCentroid[2];
    return aCentroid;
} 

std::vector<std::vector<double>> UtlCalculations::getDeviations(std::vector<std::vector<double>> A, std::vector<double> aCentroid)
{
	int n = A.size();
	std::vector<std::vector<double>> aTilda = UtlCalculations::getZeros(n);
    for (int i = 0; i < n; i++)
    {
        aTilda[i][0] = A[i][0] - aCentroid[0];
        aTilda[i][1] = A[i][1] - aCentroid[1];
        aTilda[i][2] = A[i][2] - aCentroid[2];
    }
    return aTilda;
}

std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>> UtlCalculations::qrSim3by3(std::vector<std::vector<double>> A)
{
	std::vector<std::vector<double>> Q = UtlCalculations::getZeros3by3();
    std::vector<std::vector<double>> R = UtlCalculations::getZeros3by3();

    std::vector<std::vector<double>> X = UtlCalculations::getZeros3by3();
    std::vector<std::vector<double>> Y = UtlCalculations::getZeros3by3();
    std::vector<std::vector<double>> K = UtlCalculations::getEye3by3();

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            X[i][j] = A[i][j];

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Y[i][j] = A[i][j];

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < i; j++)
        {
            K[j][i] = (X[0][i] * Y[0][j] + X[1][i] * Y[1][j] + X[2][i] * Y[2][j]) / (Y[0][j] * Y[0][j] + Y[1][j] * Y[1][j] + Y[2][j] * Y[2][j]);
            Y[0][i] = Y[0][i] - K[j][i] * Y[0][j];
            Y[1][i] = Y[1][i] - K[j][i] * Y[1][j];
            Y[2][i] = Y[2][i] - K[j][i] * Y[2][j];
        }
    }

    std::vector<double> vecY{0.0,0.0,0.0};

	for (int i = 0; i < 3; i++)
	{
		double n = (double)sqrt(Y[0][i] * Y[0][i] + Y[1][i] * Y[1][i] + Y[2][i] * Y[2][i]);
		Q[0][i] = Y[0][i] / n;
		Q[1][i] = Y[1][i] / n;
		Q[2][i] = Y[2][i] / n;
		vecY[i] = n;
	}

	std::vector<std::vector<double>> diagY = UtlCalculations::getDiag3by3(vecY);
	R = UtlCalculations::matrixMult3by3(diagY, K);

	return std::make_tuple(Q, R);
}

std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>, std::vector<std::vector<double>>> UtlCalculations::svdSim3by3(std::vector<std::vector<double>> A)
{
	std::vector<std::vector<double>> U;
	std::vector<std::vector<double>> S;
	std::vector<std::vector<double>> V;
	std::vector<std::vector<double>> Q;

	double tol = 2.2204E-16f * 1024;  // eps floating-point relative accuracy 
	int loopmax = 300;
    int loopcount = 0;

    U = UtlCalculations::getEye3by3();
    V = UtlCalculations::getEye3by3();
    S = UtlCalculations::transpose3by3(A);

    double err = std::numeric_limits<float>::max();

    while (err > tol && loopcount < loopmax)
    {
    	std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>> tulp = UtlCalculations::qrSim3by3(UtlCalculations::transpose3by3(S));
    	Q = std::get<0>(tulp);
    	S = std::get<1>(tulp);
    	U = UtlCalculations::matrixMult3by3(U, Q);

    	tulp = UtlCalculations::qrSim3by3(transpose3by3(S));
    	Q = std::get<0>(tulp);
    	S = std::get<1>(tulp);
    	V = UtlCalculations::matrixMult3by3(V, Q);

    	std::vector<std::vector<double>> e = UtlCalculations::getZeros3by3();
    	e[0][1] = S[0][1];
        e[0][2] = S[0][2];
        e[1][2] = S[1][2];

        double E = (double)sqrt(e[0][1] * e[0][1] + e[0][2] * e[0][2] + e[1][2] * e[1][2]);
        double F = (double)sqrt(S[0][0] * S[0][0] + S[1][1] * S[1][1] + S[2][2] * S[2][2]);

        if (F == 0)
            F = 1;
        err = E / F;
        loopcount++;
    }

    std::vector<double> SS{S[0][0], S[1][1], S[2][2]};
    for (int i = 0; i < 3; i++)
    {
        double SSi = abs(SS[i]);
        S[i][i] = SSi;
        if (SS[i] < 0)
        {
            U[0][i] = -U[0][i];
            U[1][i] = -U[1][i];
            U[2][i] = -U[2][i];
        }
    }

    return std::make_tuple(U, S, V);
}

bool UtlCalculations::registerPntCloud(roskst_msgs::RegisterPntCloud::Request &req, roskst_msgs::RegisterPntCloud::Response &res)
{
	int sz1 = req.pcp.fidCloud.size();
	int sz2 = req.pcp.digCloud.size();
	if(sz1!=sz2)
	{
		ROS_INFO("Fiducial and digitized points size do not agree!");
		ROS_INFO("Registration failed!");
		return true;
	}
	std::vector<std::vector<double>> a;
	std::vector<std::vector<double>> b;
	for(int i=0;i<sz1;i++)
	{
		std::vector<double> aa{
			req.pcp.fidCloud[i].x, 
			req.pcp.fidCloud[i].y, 
			req.pcp.fidCloud[i].z
		};
		std::vector<double> bb{
			req.pcp.digCloud[i].x, 
			req.pcp.digCloud[i].y, 
			req.pcp.digCloud[i].z
		};
		a.push_back(aa);
		b.push_back(bb);
	}
	std::vector<double>  a_centroid = UtlCalculations::getCentroid(a);
	std::vector<double>  b_centroid = UtlCalculations::getCentroid(b);
	std::vector<std::vector<double>> A_tilda = UtlCalculations::getDeviations(a, a_centroid);
	std::vector<std::vector<double>> B_tilda = UtlCalculations::getDeviations(b, b_centroid);
	std::vector<std::vector<double>> H = UtlCalculations::getZeros3by3();
	for(int i=0;i<sz1;i++)
	{
		H[0][0] += A_tilda[i][0]*B_tilda[i][0];
		H[0][1] += A_tilda[i][0]*B_tilda[i][1];
		H[0][2] += A_tilda[i][0]*B_tilda[i][2];
		H[1][0] += A_tilda[i][1]*B_tilda[i][0];
		H[1][1] += A_tilda[i][1]*B_tilda[i][1];
		H[1][2] += A_tilda[i][1]*B_tilda[i][2];
		H[2][0] += A_tilda[i][2]*B_tilda[i][0]; 
		H[2][1] += A_tilda[i][2]*B_tilda[i][1];
		H[2][2] += A_tilda[i][2]*B_tilda[i][2];
	}
	std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>, std::vector<std::vector<double>>> USV = UtlCalculations::svdSim3by3(H);
	std::vector<std::vector<double>> R = UtlCalculations::matrixMult3by3(std::get<2>(USV), UtlCalculations::transpose3by3(std::get<0>(USV)));
	if(UtlCalculations::det3by3(R) < 0)
	{
		std::vector<std::vector<double>> V = std::get<2>(USV);
		V[0][0] = -V[0][0]; V[0][1] = -V[0][1]; V[0][2] = -V[0][2];
        V[1][0] = -V[1][0]; V[1][1] = -V[1][1]; V[1][2] = -V[1][2];
        V[2][0] = -V[2][0]; V[2][1] = -V[2][1]; V[2][2] = -V[2][2];
        R = UtlCalculations::matrixMult3by3(V, UtlCalculations::transpose3by3(std::get<0>(USV)));
	}
	std::vector<double> p{0.0,0.0,0.0};
	p[0] = b_centroid[0] - (
	    R[0][0] * a_centroid[0] + R[0][1] * a_centroid[1] + R[0][2] * a_centroid[2]);
	p[1] = b_centroid[1] - (
	    R[1][0] * a_centroid[0] + R[1][1] * a_centroid[1] + R[1][2] * a_centroid[2]);
	p[2] = b_centroid[2] - (
	    R[2][0] * a_centroid[0] + R[2][1] * a_centroid[1] + R[2][2] * a_centroid[2]);

	geometry_msgs::Pose answ;
	answ.position.x = p[0];
	answ.position.y = p[1];
	answ.position.z = p[2];

	double qw = sqrt(1.0+R[0][0]+R[1][1]+R[2][2]) / 2.0 * 4.0;

	answ.orientation.x = (R[2][1] - R[1][2]) / qw;
	answ.orientation.y = (R[0][2] - R[2][0]) / qw;
	answ.orientation.z = (R[1][0] - R[0][1]) / qw;
	answ.orientation.w = qw/4;

	res.res = answ;

	return true;
}

std::vector<double> UtlCalculations::quat2eul(std::vector<double> q /*x,y,z,w*/)
{
    double aSinInput = -2.0 * (q[0] * q[2] - q[3] * q[1]);
    if(aSinInput > 1.0)
        aSinInput = 1.0;
    if(aSinInput < -1.0)
        aSinInput = -1.0;
    
    std::vector<double> ans{
        atan2( 2.0 * (q[0] * q[1] + q[3] * q[2]), q[3] * q[3] + q[0] * q[0] - q[1] * q[1] - q[2] * q[2] ), 
        asin( aSinInput ), 
        atan2( 2.0 * (q[1] * q[2] + q[3] * q[0]), q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2] )
    };
    return ans;
}

geometry_msgs::Quaternion UtlCalculations::eul2quat(std::vector<double> eul)
{
	geometry_msgs::Quaternion quat;
	std::vector<double> eulhalf{eul[0]/2,eul[1]/2,eul[2]/2};
    eul = eulhalf;
	quat.x = cos(eul[0]) * cos(eul[1]) * sin(eul[2]) - sin(eul[0]) * sin(eul[1]) * cos(eul[2]);
	quat.y = cos(eul[0]) * sin(eul[1]) * cos(eul[2]) + sin(eul[0]) * cos(eul[1]) * sin(eul[2]);
	quat.z = sin(eul[0]) * cos(eul[1]) * cos(eul[2]) - cos(eul[0]) * sin(eul[1]) * sin(eul[2]);
	quat.w = cos(eul[0]) * cos(eul[1]) * cos(eul[2]) + sin(eul[0]) * sin(eul[1]) * sin(eul[2]);
	return quat;
}