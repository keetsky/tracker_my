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
float positionz=1500.0;

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
    ros::Publisher human_position_pub = n.advertise<geometry_msgs::Point>("human_position", 10);
    ros::Rate loop_rate(30);
   // int count=0;


		while(!system("stty echo")&&(ros::ok))
		{       

		geometry_msgs::Point msg;
              
		nite::UserTrackerFrameRef newFrame;
		niStatus = uTracker.readFrame(&newFrame);
		if (!HandleStatus(niStatus) ||
			!newFrame.isValid()) return 1;
		//system("cls");
		const nite::Array<nite::UserData>& users =
			newFrame.getUsers();

		//float usrs_datas[5][4]={{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0}};
		float min=10000.0;
	//	int dex=0;

		int numb_usr=users.getSize();		
		printf("the numbers of users is %d",numb_usr);
		int count_0=0; //the numbers of  usrs's  distance  that equal to zero
		for (int i = 0; i < users.getSize(); ++i)
		{
		//	float posX, posY;
			float p = users[i].getCenterOfMass().z;
		//	niStatus =
		//		uTracker.convertJointCoordinatesToDepth(
		//		users[i].getCenterOfMass().x,
		//		users[i].getCenterOfMass().y,
		//		users[i].getCenterOfMass().z,
		//		&posX, &posY);
//////find the closest person ,he must be visible and the distance is the min of all person ,if none penson in the vision ,the return data is else ;
		//	if((users[i].getCenterOfMass().z<min)&&(users[i].getCenterOfMass().z>0.0)&&(users[i].isVisible()))


			
			if((p<min)&&(p>0.0))      
			{       min=users[i].getCenterOfMass().z; 
				positionx=users[i].getCenterOfMass().x;
              			positiony=users[i].getCenterOfMass().y;
                		positionz=p;	
		//		dex=i;
			}
			else
			{	count_0+=1;
				if(count_0==users.getSize())
				{
				positionx=0.0;
              			positiony=0.0;
                		positionz=1500.0;
				}	
			}
			

               // positionx=users[i].getCenterOfMass().x;
                //positiony=users[i].getCenterOfMass().y;
               // positionz=users[i].getCenterOfMass().z;
			//printf("Center penson's distance is %gmm "
			//	"located at %gmmx%gmm\r\n",
			//	users[i].getCenterOfMass().z, users[i].getCenterOfMass().x, users[i].getCenterOfMass().y);
                //printf("user=%d\n",users.getSize());
            //    printf("User #%d %s \r\n",
		//users[i].getId(),
		//(users[i].isVisible()) ? "is Visible" :
		//"is not Visible");
                       
		}
		//positionx=users[dex].getCenterOfMass().x;
                //positiony=users[dex].getCenterOfMass().y;
               // positionz=users[dex].getCenterOfMass().z;
        msg.x=positionx;    
        msg.y=positiony;
        msg.z=positionz;
	
 //      printf("Center penson's distance is %gmm "
//				"located at %gmmx%gmm\r\n",
//				msg.z, msg.x, msg.y);
	ROS_INFO("Center penson's distance is: %fmm,%fmm,%fmm",msg.z, msg.x, msg.y);
        human_position_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        //++count;

	}

	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();
	return 0;
}

