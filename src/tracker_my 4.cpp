// OpenNI2Project.cpp : Defines the entry point for the console application.
//


// General headers
#include <stdio.h>
#include <sstream>
#include <iostream>
// OpenNI2 headers
#include <OpenNI.h> 
#include <NiTE.h> 

// GLUT headers
#include <GL/glut.h>
#include <GL/gl.h>
//ros headers
#include <ros/ros.h>
#include <ros/package.h>
#include "geometry_msgs/Point.h"

using namespace openni;
using namespace std;

float positionx=0.0;
float positiony=0.0;
float positionz=0.0;

char ReadLastCharOfLine()
{
	int newChar = 0;
	int lastChar;
	fflush(stdout);
	do 
	{
		lastChar = newChar;
		newChar = getchar();
	}
	while ((newChar != '\n') 
		&& (newChar != EOF));
	return (char)lastChar;
}

bool HandleStatus(Status status)
{
	if (status == STATUS_OK)
		return true;
	printf("ERROR: #%d, %s", status,
		openni::OpenNI::getExtendedError());
	ReadLastCharOfLine();
	return false;
}

bool HandleStatus(nite::Status status)
{
	return HandleStatus((openni::Status)status);
}

int main(int argc, char** argv)
{
	Status status = STATUS_OK;
	printf("\r\n---------------------- Init OpenNI --------------------------\r\n");
	printf("Scanning machine for devices and loading "
			"modules/drivers ...\r\n");
	
	status = openni::OpenNI::initialize();
	if (!HandleStatus(status)) return 1;
	printf("Completed.\r\n");

	printf("\r\n---------------------- Open Device --------------------------\r\n");
	printf("Opening first device ...\r\n");
	openni::Device device;
	status = device.open(openni::ANY_DEVICE);
	if (!HandleStatus(status)) return 1;
	printf("%s Opened, Completed.\r\n",
		device.getDeviceInfo().getName());

	nite::Status niStatus = nite::STATUS_OK;
	printf("\r\n---------------------- Init NiTE --------------------------\r\n");
	niStatus = nite::NiTE::initialize();
	if (!HandleStatus(niStatus)) return 1;
	printf("Done\r\n");

	printf("Creating user tracker ...\r\n");
	nite::UserTracker uTracker;
	niStatus = uTracker.create(&device);
	if (!HandleStatus(niStatus)) return 1;
	printf("Reading data from user tracker ...\r\n");

ros::init(argc, argv,"tracker_my");
    ros::NodeHandle n;
    ros::Publisher human_position_pub = n.advertise<geometry_msgs::Point>("human_position", 1000);
    ros::Rate loop_rate(10);
   // int count=0;


	while(!system("stty echo")&&(ros::ok))
	{       

		geometry_msgs::Point msg;
              
		nite::UserTrackerFrameRef newFrame;
		niStatus = uTracker.readFrame(&newFrame);
		if (!HandleStatus(niStatus) ||
			!newFrame.isValid()) return 1;
		system("cls");
		const nite::Array<nite::UserData>& users =
			newFrame.getUsers();
		for (int i = 0; i < users.getSize(); ++i)
		{
			float posX, posY;
			float p = users[i].getCenterOfMass().z;
			niStatus =
				uTracker.convertJointCoordinatesToDepth(
				users[i].getCenterOfMass().x,
				users[i].getCenterOfMass().y,
				users[i].getCenterOfMass().z,
				&posX, &posY);

                positionx=users[i].getCenterOfMass().x;
                positiony=users[i].getCenterOfMass().y;
                positionz=users[i].getCenterOfMass().z;
			//printf("Center penson's distance is %gmm "
			//	"located at %gmmx%gmm\r\n",
			//	users[i].getCenterOfMass().z, users[i].getCenterOfMass().x, users[i].getCenterOfMass().y);
                printf("user=%d\n",users.getSize());
                       
		}
        msg.x=positionx;    
        msg.y=positiony;
        msg.z=positionz;
       printf("Center penson's distance is %gmm "
				"located at %gmmx%gmm\r\n",
				msg.z, msg.x, msg.y);
        human_position_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        //++count;

	}

	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();
	return 0;
}

