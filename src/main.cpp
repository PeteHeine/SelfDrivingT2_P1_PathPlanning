#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "spline.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include <stdexcept>

using namespace std;

// for convenience
using json = nlohmann::json;

struct Path {
	int lane;
	double vel;
	double cost = 0;
	vector<double> next_s_vals;
	vector<double> next_d_vals;
};

// The path of other cars are estimated using a simple constant speed assumption.
struct OtherCar{
	int lane;
	double vel;
	double s;
	double d;
};

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
	angle = min(2*pi() - angle, angle);

	if(angle > pi()/4)
	{
		closestWaypoint++;
		if (closestWaypoint == maps_x.size())
		{
			closestWaypoint = 0;
		}
	}

	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}
	frenet_s += distance(0,0,proj_x,proj_y);
	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  cout << "print stuff " << endl;

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }


  double c_lane = -1;
  // Reference velocity
  double ref_vel = 0.0; // Mph
  int count = 0;
  h.onMessage([&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &count, &c_lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	auto previous_path_yaw = j[1]["previous_path_yaw"];
          	auto previous_path_s = j[1]["previous_path_s"];
          	auto previous_path_d = j[1]["previous_path_d"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	//auto sensor_fusion = j[1]["sensor_fusion"];
          	vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];


          	bool too_close = false;
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
          	vector<double> next_s_vals;
			vector<double> next_d_vals;


			// Initialize current lane to the start lane of the vehicle.
			if(c_lane==-1)
				c_lane = round(car_d/4-0.5);

          	count = count+1;
          	cout << "ego xy: " << car_x << ", " << car_y << ", s: " << car_s << "car_yaw: " << car_yaw << ", d:" << car_d << ", speed: " << car_speed << endl;


          	// Print out stuff
          	bool po = false;

			// Max acceleration
			double max_acceleration = 0.224;

			// Defines also how quickly a car switches to another lane.
			double spacing = 50.0;

			// To select a good path, the algorithm must look many points ahead.
			int look_points_ahead_long = 150;

			int look_points_ahead_short = 30;

			// Safe zone around the car
			double safe_zone_d = 2.0;
			double safe_zone_s = 15.0;

			// Different speed settings
			double top_speed = 49.50;
			vector<double> speed_settings = {ref_vel-max_acceleration*3,ref_vel-max_acceleration,ref_vel,ref_vel+max_acceleration,ref_vel+max_acceleration*3};
			vector<double> valid_speed_interval = {0.0, top_speed};

			// Different lane change settings.
			vector<double> lane_settings = {c_lane-2,c_lane-1, c_lane, c_lane+1,c_lane+2};
			vector<double> valid_lane_interval = {0,2};


			// Keep c_lane, if vehicle already is in a lane shift.
			double lane_center = (c_lane)*4+2;
			double diff_center = abs(lane_center-car_d);
			if(diff_center>1.0){
				lane_settings = {c_lane};
			}

          	//if(count > 3) throw std::invalid_argument( "Self-thrown error !! " );

			int prev_size = previous_path_x.size();
			// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
			vector<double> ptsx;
			vector<double> ptsy;

			double ref_x = car_x;
			double ref_y = car_y;
			double ref_yaw = deg2rad(car_yaw);


			if(prev_size < 2)
			{
				double prev_car_x = car_x-cos(car_yaw);
				double prev_car_y = car_y-sin(car_yaw);

				ptsx.push_back(prev_car_x);
				ptsx.push_back(car_x);

				ptsy.push_back(prev_car_y);
				ptsy.push_back(car_y);
			}
			else
			{
				ref_x = previous_path_x[prev_size-1];
				ref_y = previous_path_y[prev_size-1];

				double ref_x_prev = previous_path_x[prev_size-2];
				double ref_y_prev = previous_path_y[prev_size-2];
				ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

				ptsx.push_back(ref_x_prev);
				ptsx.push_back(ref_x);

				ptsy.push_back(ref_y_prev);
				ptsy.push_back(ref_y);
			}

			//double c_lane = round(car_d/4-0.5);

			//////////////////////// EVALUATE PATHS ////////////////////////////////////
			// Paths are only evaluated in s and d space (previous points are ignored).

			// Define a set of potential paths
			vector<Path> paths;

			int local_count = 0;
			// While in a lane-shift. Select only the c_lane
			// Loop through lane settings
			for(int iLane = 0; iLane < lane_settings.size() ;iLane++) {

				// Only valid settings
				if((lane_settings[iLane] >= valid_lane_interval[0]) && (lane_settings[iLane] <= valid_lane_interval[1])){

					// Loop through speed settings
					for(int iSpeed = 0; iSpeed < speed_settings.size() ;iSpeed++) {

						// Invalid speed settings are rejected
						if((speed_settings[iSpeed] >= valid_speed_interval[0]) && (speed_settings[iSpeed] <= valid_speed_interval[1])){

							//cout << "1	idx(" << local_count << ")lane: " << lane_settings[iLane] << ", speed: " << speed_settings[iSpeed] << endl;
							local_count = local_count+1;
							Path tmp_path;
							tmp_path.lane = lane_settings[iLane];
							tmp_path.vel = speed_settings[iSpeed];
							paths.push_back(tmp_path);
						}
					}
				}
			}

			//cout << "Potential Paths from current path. Current lane: " << c_lane << ". Current speed: " << ref_vel << endl;


			// Create paths
			for(unsigned int iPath = 0; iPath < paths.size(); iPath++) {
			//for(unsigned int iPath = 0; iPath < 1; iPath++) {
				//cout << "2	idx(" << iPath << ")lane: " << paths[iPath].lane << ", speed: " << paths[iPath].vel << endl;

				// Create a new set of point for creating a spline??
				vector<double> pts_s;
				vector<double> pts_d;

				pts_s.push_back(car_s);
				pts_d.push_back(car_d);

				// project points forward.
				int n_points_ahead = 3;
				for(unsigned int iS = 0; iS < n_points_ahead; iS++){
					pts_s.push_back(car_s+spacing*(iS+1));
					pts_d.push_back(2+4*paths[iPath].lane);
				}

				//if(po){
					for(unsigned int iS = 0; iS < pts_s.size(); iS++){
						cout << "SplineIn (" << iS <<  ") Out: s: " << pts_s[iS] << " (" << pts_s[iS]-car_s << "), d: " << pts_d[iS] << " (" << pts_d[iS]-car_d << ")" << endl;
					}
				//}


				// Create a spline
				tk::spline spline;

				// Points are normalized by subtracting the first index.
				spline.set_points(pts_s,pts_d);


				// Calculate how to break up spline points so that we travel at our desired reference velocity
				double target_s0 = car_s;
				double target_s1 = car_s+30.0;
				double diff_target_s = target_s1-target_s0;

				double target_d0 =spline(target_s0);
				double target_d1 =spline(target_s1);
				double diff_target_d = target_d1-target_d0;
				double target_dist = sqrt((diff_target_s)*(diff_target_s)+(diff_target_d)*(diff_target_d));


				double N = (target_dist/(.02*paths[iPath].vel/2.24));

				//if (s_spacing*double(n_points_ahead))
				// Fill up the rest of our path planner after filling it with previous points. here the output will always output 50 points.
				for (int i = 1; i <= look_points_ahead_long;i++)
				{
					double s_point = target_s0+double(i)*(diff_target_s/N);
					double d_point = spline(s_point);

					if(po && (i == 1) && (look_points_ahead_long == look_points_ahead_long-1))
						cout << "SplineOut (" << i <<  ") Out: s: " << s_point << " (" << s_point-car_s<< "), d: " << d_point << " (" << d_point-car_d<< ")" << endl;

					paths[iPath].next_d_vals.push_back(d_point);
					paths[iPath].next_s_vals.push_back(s_point);
				}
			}


			// The cost is defined by the maximum length a car can move without collision
			// (This is inspired by a fellow udacity student)


			vector<OtherCar> other_cars;
			for(int iCar = 0; iCar < sensor_fusion.size();iCar++){
				vector<double> vCar = sensor_fusion[iCar];
				//cout << "sensor_fusion.size(): " << sensor_fusion.size() << ", sensor_fusion[iCar].size(): " << sensor_fusion[iCar].size() << endl;
				double other_car_s = vCar[5];
				double other_car_d = vCar[6];
				double vx = vCar[3];
				double vy = vCar[4];

				OtherCar tmp_car;
				tmp_car.vel = sqrt(vx*vx+vy*vy);
				tmp_car.s = vCar[5];
				tmp_car.d = vCar[6];
				tmp_car.lane = round(vCar[6]/4-0.5);
				other_cars.push_back(tmp_car);
			}

			for(unsigned int iPath = 0; iPath < paths.size(); iPath++) {
				Path myPath = paths[iPath];

				//cout << "	idx(" << iPath << ")lane: " << myPath.lane << ", speed: " << myPath.vel << endl;

				// Max distance traveled with no collision
				int max_idx = myPath.next_s_vals.size()-1;


				double safe_distance_max = myPath.next_s_vals[max_idx]-myPath.next_s_vals[0];
				double safe_distance = 0;

				bool collision = false;

				// Loop through all expected positions in the planned path.
				for(unsigned int iPos = 0; iPos < myPath.next_s_vals.size(); iPos++){
					double my_car_d = myPath.next_d_vals[iPos];
					double my_car_s = myPath.next_s_vals[iPos];
					double diff_d = 0;
					double diff_s = 0;
					// Loop through other cars.
					for(unsigned int iCar = 0; (iCar < other_cars.size()) && !collision; iCar++){
						//cout << "test3" << "iPath: " << iPath << ", iPos: " << iPos << ", iCar: " << iCar << endl;
						diff_d = abs(other_cars[iCar].d-my_car_d);
						// Car in other lane is safe.
						if(diff_d < safe_zone_d){
							// Distant car is also safe.
							double other_car_s = other_cars[iCar].s+double(iPos)*0.02*other_cars[iCar].vel;
							diff_s = abs(other_car_s-my_car_s);
							if(diff_s < safe_zone_s){
								collision = true;
								//cout << "Collision	idx(" << iPath << ")lane: " << paths[iPath].lane << ", speed: " << paths[iPath].vel << ", cost: " << max_distance <<  endl;
								//cout << "	other_car_s: " << other_car_s << ", my_car_s: " << my_car_s << ", other_cars[iCar].d: " << other_cars[iCar].d << ", my_car: " << my_car_d << endl;
							}
						}
					}

					if(collision){
						break;
					}
					safe_distance = myPath.next_s_vals[iPos]-myPath.next_s_vals[0];
				}
				paths[iPath].cost = (safe_distance+0.0001);
				if(collision)
					cout << "Collision	idx(" << iPath << ")lane: " << paths[iPath].lane << ", speed: " << paths[iPath].vel << ", cost: " << paths[iPath].cost << "/" << safe_distance_max <<  endl;
			}

			//cout << "test4" << endl;
			// Get the lowest cost
			double minCost = std::numeric_limits<double>::min();
			int minIdx = -1;
			for(unsigned int iPath = 0; iPath < paths.size(); iPath++) {
				if(minCost<paths[iPath].cost){
					minCost = paths[iPath].cost;
					minIdx = iPath;
				}

				//cout << "	idx(" << iPath << ")lane: " << paths[iPath].lane << ", speed: " << paths[iPath].vel << ", cost: " << paths[iPath].cost << "(" << paths[iPath].next_s_vals[paths[iPath].next_s_vals.size()-1]-paths[iPath].next_s_vals[0] << ")" <<  endl;
			}


			////////////////////////// EXECUTE PATH ///////////////////////////////////
			// Set desired path
			c_lane = paths[minIdx].lane;
			ref_vel = paths[minIdx].vel;


			// Create points ahead of vehicle
			for(unsigned int idx = 0; idx < 3; idx++) {
				vector<double> next_wp = getXY(car_s+spacing*(idx+1), (2+4*c_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
				ptsx.push_back(next_wp[0]);
				ptsy.push_back(next_wp[1]);
			}


			// "Normalize" waypoints. Points are transformed back to (0,0) in a 0-degree orientation
			for (int i = 0; i < ptsx.size();i++)
			{
				double shift_x = ptsx[i]-ref_x;
				double shift_y = ptsy[i]-ref_y;
				ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
				ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
			}

			// Create a spline
			tk::spline s;

			s.set_points(ptsx,ptsy);


			// Start with all of the previous path point from last time
			for(int i = 0; i < previous_path_x.size();i++)
			{
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}

			// Calculate how to break up spline points so that we travel at our desired reference velocity
			double target_x = 30.0;
			double target_y =s(target_x);
			double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

			double x_add_on = 0;

			double N = (target_dist/(.02*ref_vel/2.24));

			// Fill up the rest of our path planner after filling it with previous points. here the output will always output 50 points.
			for (int i = 1; i <= look_points_ahead_short-previous_path_x.size();i++)
			{
				double x_point = x_add_on+(target_x)/N;
				//double x_point = target_x*double(i);
				double y_point = s(x_point);

				x_add_on = x_point;

				double x_ref = x_point;
				double y_ref = y_point;

				// Rotate back to normal after rotating it earlier
				x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
				y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

				x_point += ref_x;
				y_point += ref_y;

				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);

			}

			json msgJson;
			// END

			msgJson["next_x"] = next_x_vals;
			msgJson["next_y"] = next_y_vals;

			auto msg = "42[\"control\","+ msgJson.dump()+"]";

			//this_thread::sleep_for(chrono::milliseconds(1000));
			ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
		}
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port: " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
