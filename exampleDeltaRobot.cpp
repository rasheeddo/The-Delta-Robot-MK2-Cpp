#include "DeltaRobotClass.h"
#include <chrono>
using namespace std::chrono;

int main()
{
	//Create an object of Delta Robot Class as DRC
	DeltaRobotClass DRC;

	//Initialize the robot and passing 3 as operation mode (Position control) as default
	DRC.RobotInit(3);

	//
	DRC.RobotTorqueON();
	DRC.GripperTorqueON();
	usleep(1000000);
	DRC.GoHome();
	usleep(1000000);
	DRC.GripperCheck();
	usleep(1000000);



	
	auto startGetRobotAng = high_resolution_clock::now();		    // start time for GetRobotAngle function
	float RobotAng[3];
	DRC.GetRobotAngle(RobotAng);
	auto stopGetRobotAng = high_resolution_clock::now();			// stop time for GetRobotAngle fuction
	printf("RobotAng1 %f \n", RobotAng[0]);
	printf("RobotAng2 %f \n", RobotAng[1]);
	printf("RobotAng3 %f \n", RobotAng[2]);
	
	auto startGetXYZ = high_resolution_clock::now();			 	// start time for GetXYZ function
	float XYZ[3];
	DRC.GetXYZ(XYZ);
	auto stopGetXYZ = high_resolution_clock::now();					// stop time for GetXYZ fuction
	printf("X %f \n", XYZ[0]);
	printf("Y %f \n", XYZ[1]);
	printf("Z %f \n", XYZ[2]);

	while(1)
	{
		DRC.GotoPoint(-500,0,-700);

		DRC.GotoPoint(-500,0,-850);

		usleep(500000);

		DRC.GripperClose();

		usleep(500000);

		DRC.GotoPoint(-500,0,-450);

		DRC.GotoPoint(0,0,-450);

		DRC.GotoPoint(0,0,-850);

		DRC.GripperOpen();

		usleep(500000);

		DRC.GotoPoint(0,0,-450);

		DRC.GotoPoint(500,0,-700);

		DRC.GotoPoint(500,0,-850);

		usleep(500000);

		DRC.GripperClose();

		usleep(500000);

		DRC.GotoPoint(500,0,-450);

		DRC.GotoPoint(0,0,-450);

		DRC.GotoPoint(0,0,-850);

		DRC.GripperOpen();

		usleep(1000000);

	}
	
	auto startGotoPoint2 = high_resolution_clock::now();			   // start time for GotoPoint function
	DRC.GotoPoint(-500,0,-800);
	auto stopGotoPoint2 = high_resolution_clock::now();			       // stop time for GotoPoint fuction

	auto startGotoPoint3 = high_resolution_clock::now();			   // start time for GotoPoint function
	DRC.GotoPoint(500,0,-800);
	auto stopGotoPoint3 = high_resolution_clock::now();			       // stop time for GotoPoint fuction

	//auto startGotoPoint4 = high_resolution_clock::now();			   // start time for GotoPoint function
	//DRC.GotoPoint(200,200,-800);
	//auto stopGotoPoint4 = high_resolution_clock::now();			       // stop time for GotoPoint fuction

	//auto startGotoPoint1 = high_resolution_clock::now();			   // start time for GotoPoint function
	//DRC.GotoPoint(200,-200,-800);
	//auto stopGotoPoint1 = high_resolution_clock::now();			       // stop time for GotoPoint fuction



	// Calculate a time used in each function (ms)
	//double GetRobotAng_period = double(duration_cast<microseconds>(stopGetRobotAng - startGetRobotAng).count());
	//double GetXYZ_period = double(duration_cast<microseconds>(stopGetXYZ - startGetXYZ).count());
	//double GotoPoint_period1 = double(duration_cast<microseconds>(stopGotoPoint1 - startGotoPoint1).count());
	double GotoPoint_period2 = double(duration_cast<microseconds>(stopGotoPoint2 - startGotoPoint2).count());
	double GotoPoint_period3 = double(duration_cast<microseconds>(stopGotoPoint3 - startGotoPoint3).count());
	//double GotoPoint_period4 = double(duration_cast<microseconds>(stopGotoPoint4 - startGotoPoint4).count());
	
	//GetRobotAng_period = GetRobotAng_period/1000000;		// time in second
	//GetXYZ_period = GetXYZ_period/1000000;
	//GotoPoint_period1 = GotoPoint_period1/1000000;
	GotoPoint_period2 = GotoPoint_period2/1000000;
	GotoPoint_period3 = GotoPoint_period3/1000000;
	//GotoPoint_period4 = GotoPoint_period4/1000000;

	//printf("Time used in GetRobotAngle(): %f [sec]\n", GetRobotAng_period);
	//printf("Time used in GetXYZ(): %f [sec]\n", GetXYZ_period);
	//printf("Time used in GotoPoint()_1: %f [sec]\n", GotoPoint_period1);
	printf("Time used in GotoPoint()_2: %f [sec]\n", GotoPoint_period2);
	printf("Time used in GotoPoint()_3: %f [sec]\n", GotoPoint_period3);
	//printf("Time used in GotoPoint()_4: %f [sec]\n", GotoPoint_period4);
	
	//exit (EXIT_FAILURE);
	//DRC.RobotTorqueOFF();
	
	// This while loop is for checking the forward and inverse kinematics of robot
	
	//while(1){

	//	DRC.KinematicsCheck();
	//}
	
	
	return(0);
}
