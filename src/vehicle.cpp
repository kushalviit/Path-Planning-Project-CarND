#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>


/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, float s,float d, float v, float a, string state) {

    this->lane = lane;
    this->s = s;
    this->current_s=s;
    this->d = d;
    this->s_v = v;
    this->s_a = a;
    this->state = state;
    max_acceleration = 1;
    this->d_v=0;
    this->d_a=0;
    this->s_j=0;
    this->d_j=0;
    this->vehicle_ahead=false;

}

Vehicle::~Vehicle() {}


vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions) {
    /*
    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
    to the next state.

    INPUT: A predictions map. This is a map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
        the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

    */
     vector<Vehicle> trajectory;
      string next_state=successor_state();
      
      trajectory=generate_trajectory(next_state,predictions);
     
    

    //return final_trajectories[best_idx];
      return trajectory;
}

string Vehicle::successor_state() {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    //cout<<"I am in successor_States"<<endl;
    string temp_state="KL";
    if(this->vehicle_ahead==false )   
        temp_state="KL"; 
    else if(this->vehicle_ahead && this->lane>0 && this->state.compare("PLCL")!=0)
        temp_state="PLCL";
    else if(this->vehicle_ahead && this->lane<(this->lanes_available-1) && this->state.compare("PLCR")!=0)
        temp_state="PLCR";
    
    //If state is "LCL" or "LCR", then just return "KL"
    return temp_state;
}

vector<Vehicle> Vehicle::generate_trajectory(string next_state, map<int, vector<Vehicle>> predictions) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    
    vector<Vehicle> trajectory;
    if (next_state.compare("CS") == 0) {
        trajectory = constant_speed_trajectory();
    } else if (next_state.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(predictions);
    } 
   else if (next_state.compare("PLCL") == 0 || next_state.compare("PLCR") == 0) {
        trajectory = prep_lane_change_trajectory(next_state, predictions);
    }
    return trajectory;
}

vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane_tc,bool &va,bool&vb) {
    /* 
    Gets next timestep kinematics (position, velocity, acceleration) 
    for a given lane. Tries to choose the maximum velocity and acceleration, 
    given other vehicle positions and accel/velocity constraints.
    */
    bool traffic_ahead;
    Vehicle ahead;
    Vehicle behind;
    float new_rv;
    va=get_vehicle_ahead(predictions,lane_tc,ahead,traffic_ahead);
     
    if(va)
    {
       
      if(get_vehicle_behind(predictions,lane_tc,behind))
        {
           vb=true;
               
                float dec=this->max_acceleration;
              if(this->ref_vel>40)
                  dec=1;

             if(this->ref_vel>ahead.ref_vel)
               new_rv=this->ref_vel-dec;
               else
             new_rv=this->ref_vel;
       }
      else{
       vb=false;
      float dec=this->max_acceleration;
      if(this->ref_vel>40)
            dec=1;

      if(this->ref_vel>ahead.ref_vel)
      new_rv=this->ref_vel-dec;
      else
      new_rv=this->ref_vel;
      }
      
    }
    else if(get_vehicle_behind(predictions,lane_tc,behind))
        {
         va=false;
          vb=true;
      
               if(this->target_speed>this->ref_vel)
     new_rv=this->ref_vel+(this->max_acceleration);
     else 
       new_rv=this->ref_vel-(this->max_acceleration);
     
        }
    else
     {
      va=false;
      vb=false;
     if(this->target_speed>this->ref_vel)
     new_rv=this->ref_vel+(this->max_acceleration);
     else 
       new_rv=this->ref_vel-(this->max_acceleration);
     
     } 
    float new_d=2+4*lane_tc;
    
  
    return{new_rv,new_d};
    
}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
    /*
    Generate a constant speed trajectory.
    */
    
    
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s+30,this->d, 5, this->s_a, this->state)};
    return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
    /*
    Generate a keep lane trajectory.
    */
    
    vector<Vehicle> trajectory ;
    bool vehi_a,vehi_b;
    vector<float> kinematics = get_kinematics(predictions, this->lane,vehi_a,vehi_b);
    float new_rv = kinematics[0];
    float new_d = kinematics[1];
    this->vehicle_ahead=vehi_a;
    for(int i=0;i<3;i++)
    {
     Vehicle temp=Vehicle(this->lane, this->s+((i+1)*25),new_d ,0.0, 0.0, "KL");
     temp.ref_vel=new_rv;
     trajectory.push_back(temp);
    }
    return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string next_state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a trajectory preparing for a lane change.
    */
    vector<Vehicle> trajectory ;
    int new_lane=this->lane+this->lane_direction[next_state];
    bool path_clear=true;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it)
    {
        for(int i=0;i<it->second.size();i++)
         {
           if(it->second[i].lane==new_lane)
            {if((it->second[i].s<this->s+35 && it->second[i].s>this->s-30) ||(it->second[i].s<this->current_s+20 && it->second[i].s>this->current_s-20 )){path_clear=false;}}
          }
     }
    if(path_clear){
        
           for(int i=0;i<3;i++)
           {
                Vehicle ahead;
                 this->vehicle_ahead=false;
                 Vehicle temp=Vehicle(new_lane, this->s+((i+1)*25),2+4*new_lane ,0.0, 0.0, "KL");
                 temp.ref_vel=this->ref_vel;
                 

                trajectory.push_back(temp);
            }
    }
    else
      {
          bool vehi_a,vehi_b;
          vector<float> kinematics = get_kinematics(predictions, this->lane,vehi_a,vehi_b);
         this->vehicle_ahead=vehi_a;
           for(int i=0;i<3;i++)
              {
                Vehicle temp=Vehicle(this->lane, this->s+((i+1)*25),2+this->lane*4 ,0.0, 0.0, next_state);
                 temp.ref_vel=kinematics[0];
               trajectory.push_back(temp);
              }
        
      }
    return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a lane change trajectory.
    */
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s+0.1, this->d,this->s_v ,this->s_a, this->state)};
    return trajectory;
}

void Vehicle::increment(int dt = 1) {
	vector<float> sd= position_at(dt);
        this->s=sd[0];
        this->d=sd[1];
}

vector<float> Vehicle::position_at(int t) {
    vector<float> sd;
    t=t*0.02;
    sd.push_back(this->s + this->s_v*t + this->s_a*t*t/2.0);
    sd.push_back(this->d+this->d_v*t+this->d_a*t*t/2.0);
    return sd;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane_tc, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    int max_s = -15;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if ((temp_vehicle.lane == lane_tc)&&((temp_vehicle.s<this->s)&& (temp_vehicle.s>this->s+max_s))){
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane_tc, Vehicle & rVehicle,bool &traffic_ahead) {
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    int min_s = this->goal_s;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
       temp_vehicle = it->second[0];
             if ((temp_vehicle.lane == lane_tc)&&(((temp_vehicle.s>this->s)&& ((temp_vehicle.s-this->s)<min_s ))||((temp_vehicle.s>this->current_s)&& ((temp_vehicle.s-this->current_s)<min_s )))){
                      min_s = temp_vehicle.s;
                   rVehicle = temp_vehicle;
                 found_vehicle = true;
             }
    }
    traffic_ahead=false;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it)
    {
        for(int i=0;i<it->second.size();i++)
         {
          if(it->second[i].s<this->s+30 && it->second[i].s>this->s){traffic_ahead=true;}
          }
     }
    return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int path_size,int horizon) {
    /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    */
	vector<Vehicle> predictions;
    for(int i = 0; i < horizon; i++) {
      vector<float> next_sd = position_at((i+1)*path_size);
      int other_lane=this->lane;
      if(next_sd[1]<4 && next_sd[1]>=0)
          other_lane=0;
       else if(next_sd[1]<8 && next_sd[1]>=4)
          other_lane=1;
       else if(next_sd[1]<=12 && next_sd[1]>=8)
            other_lane=2;
       Vehicle temp=Vehicle(other_lane, next_sd[0],next_sd[1], this->s_v, 0);
       temp.ref_vel=this->ref_vel;
      predictions.push_back(temp);
  	}
    return predictions;

}

void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
    /*
    Sets state and kinematics for ego vehicle using the last state of the trajectory.
    */
    Vehicle next_state = trajectory[1];
    this->state = next_state.state;
    this->lane = next_state.lane;
    this->s = next_state.s;
    this->s_v = next_state.s_v;
    this->s_a = next_state.s_a;
}


