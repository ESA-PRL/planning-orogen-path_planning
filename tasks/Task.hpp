/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef PATH_PLANNING_TASK_TASK_HPP
#define PATH_PLANNING_TASK_TASK_HPP

#include "path_planning/TaskBase.hpp"
#include <base/commands/Joints.hpp>
#include "path_planning/PathPlanning.hpp"
#include "fstream"

namespace envire
{
    class Environment;
    class FrameNode;
    class TraversabilityGrid;
}

namespace path_planning {

    enum PathPlanningState {WAITING, FIRST_GOAL, FIRST_POSE, FINDING_PATH, END, DEBUGGING, CLOSE_TO_GOAL};

    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
        PathPlanningState state;
        PathPlanning_lib::PathPlanning* globalPlanner;
        PathPlanning_lib::PathPlanning* localPlanner;
        PathPlanning_lib::PathPlanning* planner;
        base::samples::RigidBodyState pose;
        base::Waypoint goalWaypoint;
        base::Waypoint currentGoal;
        base::Waypoint wRover;
        base::Waypoint currentPos;
        std::vector<base::Waypoint> trajectory;
        std::vector<base::Waypoint> trajectory2D;
        std::vector< std::vector<double> > elevationMatrix;
        std::vector< std::vector<double> > costMatrix;
        std::vector< std::vector<double> > riskMatrix;
        std::vector< std::vector<double> > soilList;
        std::vector< std::vector<double> > globalCostMatrix;
        std::vector< PathPlanning_lib::terrainType* > costTable;
        base::samples::frame::Frame traversability_map;
        std::vector<double> slope_values;
        std::vector<std::string> locomotion_modes;
        std::vector<double> cost_data;
        bool pathNeedsRepair;
        bool halfTrajectory;
        bool firstIteration;
        bool calculatedGlobalWork;
        bool isArriving;
        bool isClose;
        int current_segment;
        double slip_ratio;


        // extracted from: rock-planning/planning-orogen-simple_path_globalPlanner
        RTT::FlowStatus mTraversabilityMapStatus;
        envire::Environment* mEnv;

    public:

        Task(std::string const& name = "path_planning::Task");

        Task(std::string const& name, RTT::ExecutionEngine* engine);

	      ~Task();

        bool configureHook();

        bool startHook();

        void updateHook();

        std::vector< std::vector<double> > readMatrixFile(std::string map_file);

        void readTerrainFile(std::string terrain_file, std::vector< PathPlanning_lib::terrainType* >& table);

        void errorHook();

        void stopHook();

        void cleanupHook();
    };
}

#endif
