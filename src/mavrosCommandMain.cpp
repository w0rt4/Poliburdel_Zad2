#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include "mavrosCommand.hpp"
#include <nlohmann/json.hpp>
//#include <wiringPi.h>
//#include <wiringSerial.h>
//#include "serial_port.h"
#include <thread>
#include <chrono> 
#include <SerialPort.h>
#include <SerialStream.h>
#include <unistd.h>
#include <cstring>
#include <cstdlib>

using namespace std;
using namespace LibSerial;


double latitude[65];
double longitude[65];
int pointsCount = 0;
float missionAltitude = 0.5;
int yawMapping;
#define PI 3.14159265

enum directions
{
	FromStartLine = 1,
	ToStartLine = 2
};

using json = nlohmann::json;

const uint8_t WAIT_FOR_START = 0;
const uint8_t TAKEOFF_HOME = 1;
const uint8_t NEXT_POINT = 2;
const uint8_t FLY_HOME = 3;
const uint8_t LAND_HOME = 4;
const uint8_t END = 5;
int i=0;
int currentState = 0;
bool isInit = false;
double homeLatitude;
double homeLongitude;
double homeAltitude;
bool precisionLanding = false;
double dronAltitude;


//Czestotliwosc do ustawienia w Hz
int frequency = 20;
//////////////////////////
int loopCounter;
int loopCounter1;

int checkpointsQuantity = 11;
double cordinatesPrecision = 0.00002;//0.000005;
//////////////////

void mission(mavrosCommand command);
void waitForStart(mavrosCommand command);
void takeOffHome(mavrosCommand command);
void nextPoint(mavrosCommand command);
void flyHome(mavrosCommand command);
void landHome(mavrosCommand command);
bool getCordinates(mavrosCommand command);

int main(int argc, char* argv[]){

	ros::init(argc, argv, "treasure_hunting");
	mavrosCommand command;
	
	ros::Rate loop_rate(frequency);
	sleep(1);
	
	if(getCordinates(command) == false){
		cout<<"Mission file is damaged."<<endl;
		return 0;
	}
	
	i=0;
	
	char readedChar;
	int fileDescriptor;
	size_t ms_timeout = 5000;
    string readData;
	SerialPort serial_port("/dev/ttyUSB0");
	
	try
	{
		serial_port.Open(SerialPort::BaudRate::BAUD_115200,
						 SerialPort::CharacterSize::CHAR_SIZE_8,
						 SerialPort::Parity::PARITY_NONE,
						 SerialPort::StopBits::STOP_BITS_1,
						 SerialPort::FlowControl::FLOW_CONTROL_NONE);
	}
	catch(SerialPort::AlreadyOpen)
	{
		cout << "Error during opening serial port to Arduino. Port is already open." << endl;
		return 1;
	}
	catch(SerialPort::OpenFailed)
	{
		cout << "Error during opening serial port to Arduino." << endl;
		return 1;
	}
	catch(SerialPort::UnsupportedBaudRate)
	{
		cout << "Error during opening serial port to Arduino. Unsuported baud rate." << endl;
		return 1;
	}
	catch(...)
	{
		cout << "Unknown error during opening serial port to Arduino." << endl;
		return 1;
	}
	
	cout<<"Sending initial command to metal detector." <<endl;				 
    serial_port.WriteByte('e');
    
    // kalibracja wykrywacza
    this_thread::sleep_for (chrono::seconds(10));
    cout<<"Sending reset command to metal detector." <<endl;		
    serial_port.WriteByte('a');
    
    // uruchomienie wykrywacza
    this_thread::sleep_for (chrono::seconds(1));
    cout<<"Sending turn on command to metal detector." <<endl;		
    serial_port.WriteByte('e');
    
    cout<<"Serial Port Open."<<endl;
	while (ros::ok()) 
	{
		
		/*
		uint8_t	cp;
		int result = serialPort._read_port(cp);
		if (result > 0)
		{
			if ((char)result == '\n')
			{
			}
			else
			{
				cout << cp;
			}
		}

		// Couldn't read from port
		else
		{
			cout << "ERROR: Could not read from fd %d\n"<<endl;
		}
		this_thread::sleep_for (chrono::seconds(1));
		*/
		
		while(serial_port.IsDataAvailable()) 
		{
			try
			{
				readData = serial_port.ReadLine(ms_timeout);
				cout<<readData<<endl;
				
				int value = stoi(readData);
				if(value < 50)
				{
					serial_port.WriteByte('a');
					this_thread::sleep_for (chrono::seconds(1));
				}
				
			}
			catch(...)
			{
				cout<<"Error when trying to read:" <<endl;
			}
		}
		
		if(loopCounter >= 10){
			//mission(command);
			loopCounter = 0;
		}		
		
		loopCounter++;
		//loopCounter1++;
		ros::spinOnce();
		loop_rate.sleep();
	}	
	
	try
	{
		cout<<"Turning off metal detector." <<endl;
		
		serial_port.WriteByte('f');
	}
	catch(...)
	{
		cout<<"Error when trying to turn off metal detector." <<endl;
	}
	
	serial_port.Close();
	return 0;
}

void mission(mavrosCommand command){
	switch(currentState){
		case WAIT_FOR_START:
			if(isInit == true)waitForStart(command);
			else{
				command.initSubscribers();
				isInit = true;
			}
		break;
		case TAKEOFF_HOME:
			takeOffHome(command);
		break;
		case NEXT_POINT:
			nextPoint(command);
		break;
		case LAND_HOME:
			landHome(command);
		break;
		case END:
			cout<<"END OF MISSION"<<endl;
			exit(0);
		break;
		default:

		break;
	}
}

void waitForStart(mavrosCommand command){
	
	homeLatitude = command.getGlobalPositionLatitude();
	homeLongitude = command.getGlobalPositionLongitude();
	homeAltitude = command.getGlobalPositionAltitude();
	
	latitude[pointsCount] = homeLatitude;
	longitude[pointsCount] = homeLongitude;
	
	dronAltitude = missionAltitude;
	command.guided();
	sleep(1);

	command.arm();
	sleep(1);

	command.takeOff(missionAltitude);
	currentState = TAKEOFF_HOME;
	sleep(3);
	command.flyTo(command.getGlobalPositionLatitude(), command.getGlobalPositionLongitude(), missionAltitude);
	
}

void takeOffHome(mavrosCommand command){
	
	cout<<"CURRENT ALTITUDE: "<< command.getGlobalPositionAltitude() - homeAltitude <<endl;
	if(command.getGlobalPositionAltitude() - homeAltitude >= missionAltitude){
		currentState = NEXT_POINT;
		command.flyTo(latitude[i], longitude[i], missionAltitude);
		dronAltitude = missionAltitude;
		cout<<"RIGHT ALTITUDE"<<endl;
		cout<<"FLY DESTINATION: ";
		cout<<fixed << setprecision(7) << latitude[i] <<", ";
		cout<<fixed << setprecision(7) << longitude[i] <<endl;
	
	}
	else{
		 dronAltitude = dronAltitude + 0.01;
		 command.flyTo(command.getGlobalPositionLatitude(), command.getGlobalPositionLongitude(), dronAltitude);
	 }
}

void nextPoint(mavrosCommand command){
	
	cout<<"CURRENT POSITION: ";
	cout<<fixed << setprecision(7) << command.getGlobalPositionLatitude()<<", ";
	cout<<fixed << setprecision(7) << command.getGlobalPositionLongitude()<<" ";
	cout<<" CURRENT ALTITUDE: ";
	cout<< command.getGlobalPositionAltitude() - homeAltitude<<endl;
	
	if(command.isInPosition(command.getGlobalPositionLatitude(), command.getGlobalPositionLongitude(), latitude[i], longitude[i], cordinatesPrecision)){
		
		i++;
		
		if(i >= pointsCount + 1){ //IS IN HOME POSITION?
			currentState = LAND_HOME;
			dronAltitude = 5;
			command.flyTo(homeLatitude, homeLongitude, dronAltitude);
			return;
		}
		
		command.flyTo(latitude[i], longitude[i], missionAltitude);
	}
}

void landHome(mavrosCommand command){
	
	if(!precisionLanding){
		
		cout<<"CURRENT ALTITUDE: "<< command.getGlobalPositionAltitude() - homeAltitude<<endl;	
		if(command.getGlobalPositionAltitude() - homeAltitude <= 5){
			precisionLanding = true;
			sleep(3);
			command.land();
		}
		else{
			 dronAltitude = dronAltitude - 0.1;
			 command.flyTo(homeLatitude, homeLongitude, dronAltitude);
		 }
	}
	else{
		cout<<command.getArmed()<<endl;
		if(!command.getArmed())currentState = END;
	}
}

bool getCordinates(mavrosCommand command){
	ifstream theFile("/home/odroid/catkin_ws/src/treasure_hunting/mission.json");
	json missionSettings = json::parse(theFile);
	theFile.close();
	
	int ik,jk;
 	double x, x_wsp_14, x_wsp_12, x_pom;
 	double y, y_wsp_14, y_wsp_12, y_pom;
 	double easting, northing, longitudeShift, latitudeShift;
 	int zone;
 	bool northp;
 	
 	missionAltitude = missionSettings["mission"]["altitude"];
 	
 	double leftDownLongitude = missionSettings["mission"]["leftDown"]["longitude"];
 	double leftDownLatitude = missionSettings["mission"]["leftDown"]["latitude"];
 	double leftUpLongitude = missionSettings["mission"]["leftUp"]["longitude"];
 	double leftUpLatitude = missionSettings["mission"]["leftUp"]["latitude"];
 	double rightDownLongitude = missionSettings["mission"]["rightDown"]["longitude"];
 	double rightDownLatitude = missionSettings["mission"]["rightDown"]["latitude"];
 	
 	int direction = directions(FromStartLine);
 	
 	int pointsOnSingleScan = 8;
 	int scanCount = 8;
 	
	if(missionAltitude == 0 || leftDownLatitude == 0 || leftUpLatitude == 0 || rightDownLatitude == 0 || leftDownLongitude == 0 || leftUpLongitude == 0 || rightDownLongitude == 0)
	{
		return false;
	}	 
	
	yawMapping = atan((leftUpLongitude - leftDownLongitude) * 0.67 / (leftUpLatitude - leftDownLatitude) * 1.11) * 180 / PI;

	if(leftUpLongitude - leftDownLongitude >= 0 && leftUpLatitude - leftDownLatitude == 0)
	{
		yawMapping = 90;
	}
	else if(leftUpLatitude - leftDownLatitude < 0)
	{
		yawMapping = 180 + yawMapping;
	}
	else if(leftUpLongitude - leftDownLongitude < 0  && leftUpLatitude - leftDownLatitude == 0)
	{
		yawMapping = 270;
	}
	else if(leftUpLongitude - leftDownLongitude < 0  && leftUpLatitude - leftDownLatitude > 0)
	{
		yawMapping = 360 + yawMapping;
	}
	
	yawMapping = yawMapping % 360;
				
 	x_wsp_12 = leftUpLatitude - leftDownLatitude;
	y_wsp_12 = leftUpLongitude - leftDownLongitude;
	x_wsp_14 = rightDownLatitude - leftDownLatitude;
	y_wsp_14 = rightDownLongitude - leftDownLongitude;
	x_wsp_12 = x_wsp_12 / (pointsOnSingleScan - 1);
	y_wsp_12 = y_wsp_12 / (pointsOnSingleScan - 1);
	x_wsp_14 = x_wsp_14 / (scanCount - 1);
	y_wsp_14 = y_wsp_14 / (scanCount - 1);
	x_pom = leftDownLatitude;
	y_pom = leftDownLongitude;
 	
 	for(jk = 0; jk < scanCount; jk++)
 	{
		if(direction == directions(FromStartLine))
		{
			x = x_pom;
			y = y_pom;
			latitude[pointsCount] = x;
			longitude[pointsCount] = y;
			pointsCount++;
			
			for(ik = 1; ik < pointsOnSingleScan; ik++)
			{
				x = x_pom + ik * x_wsp_12;
				y = y_pom + ik * y_wsp_12;
				latitude[pointsCount] = x;
				longitude[pointsCount] = y;
				pointsCount++;
			}
			
			x_pom = x + x_wsp_14;
			y_pom = y + y_wsp_14;
			
			direction = directions(ToStartLine);
	   }
		else if(direction == directions(ToStartLine))
		{
			x = x_pom;
			y = y_pom;
			latitude[pointsCount] = x;
			longitude[pointsCount] = y;
			pointsCount++;
			
			for(ik = 1; ik < pointsOnSingleScan; ik++)
			{
				x = x_pom - ik * x_wsp_12;
				y = y_pom - ik * y_wsp_12;
				latitude[pointsCount] = x;
				longitude[pointsCount] = y;
				pointsCount++;
			}
			
			x_pom = x + x_wsp_14;
			y_pom = y + y_wsp_14;
			direction = directions(FromStartLine);
	   }
	}
	
	return true;
}


