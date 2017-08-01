/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef PATH_PLANNING_TASK_TASK_HPP
#define PATH_PLANNING_TASK_TASK_HPP

#include "path_planning/TaskBase.hpp"
#include "../../path_planning/src/PathPlanning.hpp"
#include "fstream"

using namespace PathPlanning_lib;

namespace envire {
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
        base::samples::RigidBodyState pose;
        base::Waypoint goalWaypoint;
        base::Waypoint currentGoal;
        base::Waypoint wRover;
        base::Waypoint currentPos;
        std::vector<base::Waypoint> trajectory;
        std::vector<short int> locVector;
        std::vector< std::vector<double> > elevationMatrix;
        std::vector< std::vector<double> > costMatrix;
        std::vector< std::vector<double> > riskMatrix;
        std::vector< std::vector<double> > soilList;
        std::vector< std::vector<double> > globalCostMatrix;
        PathPlanning_lib::NodeMap* map;
        PathPlanning_lib::NodeMap* localNodeMap;
        PathPlanning_lib::NodeMap* globalMap;
        envire::ElevationGrid* workGrid;
        envire::TraversabilityGrid* stateGrid;
        bool newVisibleArea;
        bool halfTrajectory;
        bool firstIteration;
        bool calculatedGlobalWork;
        bool isArriving;
        int current_segment;

        base::commands::Joints ptu_joints_commands_out;

        // extracted from: rock-planning/planning-orogen-simple_path_globalPlanner
        RTT::FlowStatus mTraversabilityMapStatus;
        envire::Environment* mEnv;

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "path_planning::Task");

        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices.
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
         *
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	      ~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        std::vector< std::vector<double> > readMatrixFile(std::string map_file);
        std::vector< std::vector<double> > readTerrainFile(std::string terrain_file);

        envire::Environment* matrix2envire(PathPlanning_lib::NodeMap * map);

        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        RTT::FlowStatus receiveEnvireData();
        bool extractTraversability();
    };
}

#endif
