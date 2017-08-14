#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

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
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
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

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
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

  // starting lane
  int lane = 1;
  // reference velocity
  double ref_vel = 0; // zero for cold start 49.5;	// close to speed limit, with a bit of leeway


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          	// Print some info to help with dev
          	//std::cout << " Car position at present, s=" << car_s << ", d=" << car_d << std::endl;

          	// get the size of the previous path (if any)
          	int prev_size = previous_path_x.size();

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

          	/*
          	// Sample code to just send a straight line to the simulator
          	double dist_inc = 0.5;
            for(int i = 0; i < 50; i++)
            {
                  next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
                  next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
            }*/

/*
            // Sample code to send the car in circles
            double pos_x;
            double pos_y;
            double angle;
            int path_size = previous_path_x.size();

            for(int i = 0; i < path_size; i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            if(path_size == 0)
            {
                pos_x = car_x;
                pos_y = car_y;
                angle = deg2rad(car_yaw);
            }
            else
            {
                pos_x = previous_path_x[path_size-1];
                pos_y = previous_path_y[path_size-1];

                double pos_x2 = previous_path_x[path_size-2];
                double pos_y2 = previous_path_y[path_size-2];
                angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
            }

            double dist_inc = 0.5;
            for(int i = 0; i < 50-path_size; i++)
            {
                next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
                next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
                pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
                pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
            }
            // End of circles code
*/

/*          	// Can we just send the car along the current lane?
          	// This code creates a jagged path
          	double dist_inc = 0.5;
          	vector<double> new_xy;
          	for(int i = 0; i < 50; i++)
          	{
          		new_xy = getXY(car_s + (dist_inc * i), car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          		next_x_vals.push_back(new_xy[0]);
          		next_y_vals.push_back(new_xy[1]);
          	}*/


          	// Avoiding other cars
          	if(prev_size > 0)
          	{
          		car_s = end_path_s;				// does this screw up the planning of the keep-lane path?
          	}

          	bool too_close = false;

          	// find ref_v to use
          	for (int i = 0; i < sensor_fusion.size(); i++)
          	{
          		// car is in my lane
          		float d = sensor_fusion[i][6];
          		if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2))
          		{

          			double vx = sensor_fusion[i][3];
          			double vy = sensor_fusion[i][4];
          			double check_speed = sqrt(vx*vx+vy*vy);
          			double check_car_s = sensor_fusion[i][5];

          			check_car_s += ((double)prev_size * .02 * check_speed);  // can project s value using previous points

          			// check if s value is greater than my car and s gap
          			if((check_car_s > car_s) && ((check_car_s - car_s) < 30))	// within 30m...
          			{
          				// logic here to lower ref velocity, so we don't rear-end the car in front
          				// start to consider changing lanes as well
          				//ref_vel = 29.5;  // in mph
          								 // just setting this slows the car down ... permanently - it never speeds up again...
          								 // will also need to consider not doing a jerk deceleration

          				too_close = true;
/*          				if(lane > 0)				// just jump into the left lane, for demo...
          				{
          					lane = 0;				// spline calc will sort this out later...
          				}*/
          			}
          		}

          	}

          	// slow down if too close to the car in front
          	if(too_close)
          	{
          		ref_vel -= 0.224;			// why this value?  Approx. 5m/s2 (under the 10m/s2 requirement)
          	}
          	else if(ref_vel < 49.5)			// less than max speed
          	{
          		ref_vel += 0.224;
          	}

          	// score the lanes using a cost function and pick one to move into
          	if (too_close)
          	{
				// first weighting parameter - distance to a car ahead in this lane
				double weight_car_ahead = -10.0;

				int best_lane;
				double best_lane_score = -1e6;		// really big negative number
				double this_lane_score = 0;
				for (int l = 2; l >=0; l--)			// bias to towards turn-off lane (UK legal requirement to drive on the left-most, transposed to US road layout... ;-) )
				{
					// score the l-th lane
					this_lane_score = 0;
					for (int i = 0; i < sensor_fusion.size(); i++)
					{
						// are any cars in this lane l and within 30m of our car
						float d = sensor_fusion[i][6];	// d-value of the i-th car
						if (d < (2 + 4 * l + 2) && d > (2 + 4 * l - 2))   // is this car in the l-th lane?
						{
							// yes, in the lane we're looking at...
							double vx = sensor_fusion[i][3];
							double vy = sensor_fusion[i][4];
							double check_speed = sqrt(vx*vx+vy*vy);
							double check_car_s = sensor_fusion[i][5];

							check_car_s += ((double)prev_size * .02 * check_speed);  // can project s value using previous points, same as for in-lane checking
							if((check_car_s > car_s) && ((check_car_s - car_s) < 30))	// within 30m...
							{
								// yes, within the lane we're looking at
								std::cout << "Found car (" << i << ") in lane " << l << " at distance " << (check_car_s - car_s) << std::endl;
								// penalise this lane
								this_lane_score += ((30 - (check_car_s - car_s)) * weight_car_ahead);		// give some penalty proportionate to the nearness of the car in this lane

							}
						}
					}
					if (this_lane_score > best_lane_score)
					{
						// better lane than any so far...
						// check that we're not doing a 2-lane hop or this will break the jerk requirement...
						if (abs(lane - l) <= 1)
						{
							best_lane = l;
							best_lane_score = this_lane_score;
						}
						else
						{
							std::cout << "Discarding 2-lane hop from " << lane << " to " << l << std::endl;
						}
					}
					std::cout << "Lane " << l << " score " << this_lane_score << " (best = " << best_lane_score << ")" << std::endl;
				}
				std::cout << "Best lane option is: " << best_lane << std::endl;
				lane = best_lane;
          	}

          	// check if the chosen lane is safe to move into
          	// once a lane is selected to move into, the car should complete the lane-change before choosing a new path


          	// The remaining code creates a smooth path by adding new points to the previous un-used points
          	// This code will respect the setting for "lane" and aim to drive the car into that lane
          	// So we need to be sure that this is the desired lane and that it is safe to move to that lane (or remain in it)

          	// how to create a smooth path (spline) over the range of the required path
          	vector<double> ptsx;
          	vector<double> ptsy;
          	double ref_x = car_x;
          	double ref_y = car_y;
          	double ref_yaw = deg2rad(car_yaw);

          	if (prev_size < 2)
          	{
          		// use 2 points tangential to the car

          		std::cout << "Less than 2 previous points" << std::endl;

          		double prev_car_x = car_x - cos(car_yaw);
          		double prev_car_y = car_y - sin(car_yaw);
          		ptsx.push_back(prev_car_x);
          		ptsx.push_back(car_x);
          		ptsy.push_back(prev_car_y);
          		ptsy.push_back(car_y);

          		std::cout << "Points: " << prev_car_x << "," << prev_car_y << " and " << car_x << ","<< car_y << std::endl;
          	}
          	else
          	{
          		// redefine state as previous path end point
          		ref_x = previous_path_x[prev_size - 1];
          		ref_y = previous_path_y[prev_size - 1];

          		double ref_x_prev = previous_path_x[prev_size - 2];
          		double ref_y_prev = previous_path_y[prev_size - 2];
          		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

          		ptsx.push_back(ref_x_prev);
          		ptsx.push_back(ref_x);

          		ptsy.push_back(ref_y_prev);
          		ptsy.push_back(ref_y);

          	}

          	// now add some points a distance apart down the road
          	double point_distance = 30.0;
          	vector<double> next_wp0 = getXY(car_s + point_distance, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> next_wp1 = getXY(car_s + 2 * point_distance, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> next_wp2 = getXY(car_s + 3 * point_distance, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          	ptsx.push_back(next_wp0[0]);
          	ptsx.push_back(next_wp1[0]);
          	ptsx.push_back(next_wp2[0]);

          	ptsy.push_back(next_wp0[1]);
          	ptsy.push_back(next_wp1[1]);
          	ptsy.push_back(next_wp2[1]);

          	// shift car reference angle to 0 degrees
          	for (int i = 0; i < ptsx.size(); i++)
          	{
          		double shift_x = ptsx[i] - ref_x;
          		double shift_y = ptsy[i] - ref_y;
          		ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
          		ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          	}

/*
          	std::cout << "Vector sizes: " << ptsx.size() << "," << ptsy.size() << std::endl;
          	for (int i = 0; i < ptsx.size(); i++)
          	{
          		std::cout << ptsx[i] << "," << ptsy[i] << std::endl;
          	}
*/

          	// create a spline
          	tk::spline s;

          	// set (x,y) points to the spline
          	s.set_points(ptsx, ptsy);

          	// start with all of the previous points from the last path
          	for (int i = 0; i < previous_path_x.size(); i++)
          	{
          		next_x_vals.push_back(previous_path_x[i]);
          		next_y_vals.push_back(previous_path_y[i]);
          	}
          	//std::cout << "Added " << previous_path_x.size() << " points from previous path" << std::endl;

          	// calculate how to break up the spline points to travel at our required velocity

          	// TODO: how does this add points to the *end* of the previous path (i.e. where is the transformation of the origin?)

          	double target_x = 30.0;
          	double target_y = s(target_x);
          	double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
          	double x_add_on = 0;

          	for (int i = 1; i <= 50 - previous_path_x.size(); i++)
          	{
          		double N = (target_dist / (.02 * ref_vel / 2.24));			// Divide by 2.24 to go from MPH to M/S
          		double x_point = x_add_on + (target_x) / N;
          		double y_point = s(x_point);

          		x_add_on = x_point;

          		double x_ref = x_point;
          		double y_ref = y_point;

          		// rotate back to reverse the transformation done earlier
          		x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
          		y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

          		x_point += ref_x;
          		y_point += ref_y;

          		next_x_vals.push_back(x_point);
          		next_y_vals.push_back(y_point);

          	}
          	//std::cout << "Added " << 50 - previous_path_x.size() << " new points" << std::endl;

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
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
















































































