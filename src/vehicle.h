#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

  map<string, int> lane_direction = {{"PLCL", -1}, {"PLCR", 1}};

  bool vehicle_ahead;
 

  int lane;
  float ref_vel;
  float current_s;

  float s;// Frenet coordinate s of vehicle
   
  float d; // Frenet coordinate d of vehicle

  float s_v; // velocity in s direction

  float s_a; // acceleration in s direction
  
  float s_j; // jerk in s direction
  
  float d_v; // velocity in d direction
  
  float d_a; // acceleration in d direction

  float d_j; // jerk in d direction

  float target_speed;

  int lanes_available;

  float max_acceleration;

  int goal_lane;

  float goal_s;

  string state;

  /**
  * Constructor
  */
  Vehicle();
  Vehicle(int lane, float s,float d,float v, float a, string state="CS");

  /**
  * Destructor
  */
  virtual ~Vehicle();

  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);

  string successor_state();

  vector<Vehicle> generate_trajectory(string next_state, map<int, vector<Vehicle>> predictions);

  vector<float> get_kinematics(map<int, vector<Vehicle>> predictions, int lane,bool &va,bool&vb);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);

  vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<Vehicle> prep_lane_change_trajectory(string next_state, map<int, vector<Vehicle>> predictions);

  void increment(int dt);

  vector<float> position_at(int t);

  bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane_tc, Vehicle & rVehicle);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane_tc, Vehicle & rVehicle,bool &traffic_ahead);

  vector<Vehicle> generate_predictions(int path_size,int horizon=2);

  void realize_next_state(vector<Vehicle> trajectory);

  
};

#endif
