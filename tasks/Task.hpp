#pragma once

#include <base/commands/Joints.hpp>
#include "base-logging/Logging.hpp"
#include "fstream"
#include "iostream"
#include "path_planning/DyMu.hpp"
#include "path_planning/TaskBase.hpp"

namespace path_planning
{

class Task : public TaskBase
{
    friend class TaskBase;

  protected:
    PathPlanning_lib::DyMuPathPlanner* planner;
    base::samples::RigidBodyState pose;
    base::Waypoint goalWaypoint;
    base::Waypoint currentGoal;
    base::Waypoint wRover;
    base::Waypoint currentPos;
    std::vector<base::Waypoint> trajectory;
    std::vector<base::Waypoint> trajectory2D;
    std::vector<std::vector<double>> elevationMatrix;
    std::vector<std::vector<double>> terrain_matrix;
    std::vector<std::vector<double>> costMatrix;
    std::vector<std::vector<double>> risk_matrix;
    std::vector<std::vector<double>> deviation_matrix;
    std::vector<std::vector<double>> hazard_density_matrix;
    std::vector<std::vector<double>> trafficability_matrix;
    std::vector<std::vector<double>> soilList;
    std::vector<std::vector<double>> global_cost_matrix;
    std::vector< std::vector<double> > total_cost_matrix;
    base::samples::frame::Frame traversability_map;
    std::vector<double> slope_values;
    std::vector<double> global_offset;
    std::vector<std::string> locomotion_modes;
    std::vector<double> cost_data;
    double risk_distance;
    double risk_ratio;
    double reconnect_distance;
    bool isArriving;
    bool isClose;
    bool set_travmap;
    int current_segment;
    double slip_ratio;
    base::Time local_computation_time;
    std::ofstream localTimeFile;
    std::ofstream total_cost_file;
    std::ofstream global_cost_file;
    std::ofstream path_file;
    std::ofstream risk_file;
    std::ofstream deviation_file;
    std::ofstream hazard_density_file;
    std::ofstream trafficability_file;
    PathPlanning_lib::repairingAproach input_approach;
	int num_terrains;
	int num_criteria;
	std::vector<double> weights;
	std::vector<double> feedback_data;

  public:
    Task(std::string const& name = "path_planning::Task");
    Task(std::string const& name, RTT::ExecutionEngine* engine);
    ~Task();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
    void writeGlobalResults();
    void writeLocalResults();
    std::vector<std::vector<double>> readMatrixFile(std::string map_file);
  // Registration of numer of times Global Path Planning has been successfully
  // executed
    uint num_globalpp_executions;
};
}  // namespace path_planning
