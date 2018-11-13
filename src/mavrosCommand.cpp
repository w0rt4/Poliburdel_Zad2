#include <iostream>
#include "mavrosCommand.hpp"

using namespace std;
using namespace cv;
#define PI 3.14159265

mavrosCommand::mavrosCommand() {

	_nh = ros::NodeHandle();
	init();
}

string get_username() {
	struct passwd *pwd = getpwuid(getuid());
	if (pwd)
		return pwd->pw_name;
	else
		return "odroid";
}

mavrosCommand::~mavrosCommand() {
}

void mavrosCommand::init() {
	_clientArming = _nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	_clientTakeOff = _nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
	_clientGuided = _nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
	_clientLand = _nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
	_clientServo = _nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
	_clientParamSet = _nh.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");

	_pub_mav = _nh.advertise<mavros_msgs::GlobalPositionTarget>("/mavros/setpoint_raw/global", 100);
	_pub_mavPositionTarget = _nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

	_compassHeadingSub = _nh.subscribe("/mavros/global_position/compass_hdg", 100, &mavrosCommand::compassHeadingCb, this);
	_adsbVehicleSub = _nh.subscribe("/mavros/adsb/vehicle", 100, &mavrosCommand::adsbVehicleCb, this);
	_globalPositionGlobalSub = _nh.subscribe("/mavros/global_position/global", 100, &mavrosCommand::globalPositionGlobalCb, this);
	_stateSub = _nh.subscribe("/mavros/state", 100, &mavrosCommand::stateCb, this);
	_globalPositionRelAltitudeSub = _nh.subscribe("/mavros/global_position/rel_alt", 100, &mavrosCommand::globalPostionRelAltitudeCb, this);
	_timeReferenceSub = _nh.subscribe("/mavros/time_reference", 100, &mavrosCommand::timeReferenceCb, this);
}



void mavrosCommand::adsbVehicleCb(mavros_msgs::ADSBVehicle::ConstPtr msg) {
	_adsbICAO = msg->ICAO_address;
	_adsbHeading = msg->heading;
	_adsbVelocity = msg->hor_velocity;
	_adsbLatitude = msg->latitude;
	_adsbLongitude = msg->longitude;

}

void mavrosCommand::globalPositionGlobalCb(sensor_msgs::NavSatFix::ConstPtr msg) {
	_globalPositionLatitude = msg->latitude;
	_globalPositionLongitude = msg->longitude;
	_globalPositionAltitude = msg->altitude;
}

void mavrosCommand::compassHeadingCb(std_msgs::Float64::ConstPtr msg) {
	_compassHeading = msg->data;
}

void mavrosCommand::stateCb(mavros_msgs::State::ConstPtr msg) {
	_connected = msg->connected;
	_armed = msg->armed;
	_guided = msg->guided;
	_state = msg->mode;
}

void mavrosCommand::globalPostionRelAltitudeCb(std_msgs::Float64::ConstPtr msg) {
	_relativeAltitude = msg->data;
}

void mavrosCommand::timeReferenceCb(sensor_msgs::TimeReference::ConstPtr msg) {
	_time = msg->time_ref.toSec();
}

void mavrosCommand::takeOff(double altitude) {

	mavros_msgs::CommandTOL srv_takeOff;
	srv_takeOff.request.min_pitch = 0.0;
	srv_takeOff.request.yaw = 0.0;
	srv_takeOff.request.latitude = 0.0;
	srv_takeOff.request.longitude = 0.0;
	srv_takeOff.request.altitude = altitude;
	_clientTakeOff.call(srv_takeOff);
	if (srv_takeOff.response.success) {
		cout << "TAKE OFF SUCCESFUL" << endl;
	}
	else cout << "TAKE OFF FAIL" << endl;
}

void mavrosCommand::land() {

	mavros_msgs::CommandTOL srv_land;
	srv_land.request.min_pitch = 0.0;
	srv_land.request.yaw = 0.0;
	srv_land.request.latitude = 0.0;
	srv_land.request.longitude = 0.0;
	srv_land.request.altitude = 0.0;
	_clientLand.call(srv_land);
	if (srv_land.response.success) {
		cout << "LAND SUCCEFUL" << endl;
	}
	else cout << "LAND FAIL" << endl;

}

bool mavrosCommand::servo(double width) {//width 1000-2000

	mavros_msgs::CommandLong srv_servo;
	srv_servo.request.command = 183;
	srv_servo.request.param1 = 9.0;
	srv_servo.request.param2 = width;
	_clientServo.call(srv_servo);
	if (srv_servo.response.success)
	{
		cout << "SERVO SUCCESFUL" << endl;
		return true;
	}

	cout << "SERVO FAIL" << endl;
	return false;
}

bool mavrosCommand::guided() {

	mavros_msgs::SetMode srvSetMode;
	srvSetMode.request.custom_mode = "GUIDED";
	_clientGuided.call(srvSetMode);
	if (srvSetMode.response.mode_sent)
	{
		cout << "GUIDED MODE SUCCESFUL" << endl;
		return true;
	}

	cout << "GUIDED MODE FAIL" << endl;
	return false;
}

bool mavrosCommand::arm()
{
	mavros_msgs::CommandBool srv;

	for (int i = 0; i < 3; i++)
	{
		srv.request.value = true;
		_clientArming.call(srv);

		if (srv.response.success)
		{
			cout << "ARM SUCCESFUL" << endl;
			return true;
		}
		else
		{
			cout << "ARM FAIL" << endl;
			sleep(5);
		}
	}

	return false;
}

void mavrosCommand::flyTo(double latitude, double longitude, double altitude) {

	cmd_pos_glo.header.frame_id = "SET_POSITION_TARGET_GLOBAL_INT";
	cmd_pos_glo.coordinate_frame = 6;
	cmd_pos_glo.type_mask = 4088;
	cmd_pos_glo.latitude = latitude;
	cmd_pos_glo.longitude = longitude;
	cmd_pos_glo.altitude = altitude;
	cmd_pos_glo.velocity.x = 0.0;
	cmd_pos_glo.velocity.y = 0.0;
	cmd_pos_glo.velocity.z = 0.0;
	cmd_pos_glo.acceleration_or_force.x = 0.0;
	cmd_pos_glo.acceleration_or_force.y = 0.0;
	cmd_pos_glo.acceleration_or_force.z = 0.0;
	cmd_pos_glo.yaw = 0.0;
	cmd_pos_glo.yaw_rate = 0.0;

	_pub_mav.publish(cmd_pos_glo);
}

void mavrosCommand::flyToLocal(double forward, double right, double up, float yaw) {

	double yaw_rad = toRad(yaw) + PI / 2;
	cmd_pos_target.header.frame_id = "SET_POSITION_TARGET_LOCAL_NED";
	cmd_pos_target.coordinate_frame = 9;
	cmd_pos_target.type_mask = 3064;
	cmd_pos_target.position.x = right;
	cmd_pos_target.position.y = forward;
	cmd_pos_target.position.z = up;
	cmd_pos_target.yaw = yaw_rad;

	_pub_mavPositionTarget.publish(cmd_pos_target);
}

bool mavrosCommand::speedSet(float speed)
{
	if (speed < 20.0)
	{
		cout << "SPEED TOO LOW" << endl;
		return false;
	}
	else if (speed > 20000.0)
	{
		cout << "SPEED TOO HIGH" << endl;
		return false;
	}

	mavros_msgs::ParamSet srv_setSpeed;
	srv_setSpeed.request.param_id = "WPNAV_SPEED";
	//srv_setSpeed.request.value.integer = speed;
	srv_setSpeed.request.value.real = speed;

	_clientParamSet.call(srv_setSpeed);
	
	if (srv_setSpeed.response.success)
	{
		cout << "SET SPEED SUCCESFUL" << endl;
		return true;
	}

	cout << "SET SPEED FAIL" << endl;
	return false;
}

double mavrosCommand::getCompassHeading() {
	return _compassHeading;
}
int mavrosCommand::getTime() {
	return _time;
}
double mavrosCommand::getRelativeAltitude() {
	return _relativeAltitude;
}

int mavrosCommand::getAdsbIcao() {
	return _adsbICAO;
}
double mavrosCommand::getAdsbHeading() {
	return _adsbHeading;
}
double mavrosCommand::getAdsbVelocity() {
	return _adsbVelocity;
}
double mavrosCommand::getAdsbLatitude() {
	return _adsbLatitude;
}
double mavrosCommand::getAdsbLongitude() {
	return _adsbLongitude;
}

double mavrosCommand::getGlobalPositionLatitude() {
	return _globalPositionLatitude;
}
double mavrosCommand::getGlobalPositionLongitude() {
	return _globalPositionLongitude;
}
double mavrosCommand::getGlobalPositionAltitude() {
	return _globalPositionAltitude;
}

bool mavrosCommand::getConnected() {
	return _connected;
}
bool mavrosCommand::getArmed() {
	return _armed;
}
bool mavrosCommand::getGuided() {
	return _guided;
}
string mavrosCommand::getState() {
	return _state;
}


//others
double mavrosCommand::toRad(double degree) {
	return degree / 180 * PI;
}

bool mavrosCommand::isInPosition(double lat_current, double long_current, double lat_destination, double long_destination, double cordinatesPrecision) {

	if (lat_current >= lat_destination - cordinatesPrecision &&
		lat_current <= lat_destination + cordinatesPrecision &&
		long_current >= long_destination - cordinatesPrecision &&
		long_current <= long_destination + cordinatesPrecision) {
		cout << "IN THE RIGHT POSITION" << endl;
		return true;
	}
	else {
		cout << "STILL FLYING" << endl;
		return false;
	}
}

double mavrosCommand::distanceBetweenCordinates(double lat1, double long1, double lat2, double long2) {

	double dist;
	dist = sin(toRad(lat1)) * sin(toRad(lat2)) + cos(toRad(lat1)) * cos(toRad(lat2)) * cos(toRad(long1 - long2));
	dist = acos(dist);
	dist = 6371 * dist * 1000;
	return dist;
}

void mavrosCommand::initSubscribers() {
	getTime();
	getGlobalPositionLatitude();
	getGlobalPositionLongitude();
	getGlobalPositionAltitude();
	getArmed();
	getCompassHeading();
	getState();
}

void MyLine::calculate_a_b(MyPoint P1, MyPoint P2) {
	a = (P2.y - P1.y) / (P2.x - P1.x);
	b = P1.y - a * P1.x;
}

double MyLine::LPDist(MyPoint P) {

	double a2 = -a;
	double b2 = P.y - a2 * P.x;
	double xp = (b2 - b) / (a - a2);
	double yp = a * xp + b;
	double dx = P.x - xp;
	double dy = P.y - yp;

	return sqrt(dx*dx + dy * dy);
}

double newRange(double x, double in_min, double in_max, double out_min, double out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool neighborhoodCheck(Mat z, int x, int y) { // sprawdza cz piksel ma mniejszych sasiadow, czy jest największym pikselem z sąsiedctwa
	bool n = false;
	if (
		z.at<uint16_t>(y, x) > z.at<uint16_t>(y - 1, x - 1) and
		z.at<uint16_t>(y, x) > z.at<uint16_t>(y - 1, x) and
		z.at<uint16_t>(y, x) > z.at<uint16_t>(y - 1, x + 1) and
		z.at<uint16_t>(y, x) > z.at<uint16_t>(y, x - 1) and
		z.at<uint16_t>(y, x) > z.at<uint16_t>(y, x + 1) and
		z.at<uint16_t>(y, x) > z.at<uint16_t>(y + 1, x - 1) and
		z.at<uint16_t>(y, x) > z.at<uint16_t>(y + 1, x) and
		z.at<uint16_t>(y, x) > z.at<uint16_t>(y + 1, x + 1))
	{
		n = true;
	}

	return n;
}

bool compareLocalMax(LocalMax i, LocalMax j)
{
	return (i.value > j.value);
}

LocalMax::LocalMax(int x1, int y1, unsigned int value1)
{
	x = x1;
	y = y1;
	value = value1;
	isBiggest = true;
}

