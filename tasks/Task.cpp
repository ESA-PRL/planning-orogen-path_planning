/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace path_planning;
using namespace Eigen;
namespace LM = locomotion_switcher;

Task::Task(std::string const& name)
    : TaskBase(name),
      mEnv(NULL)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine),
      mEnv(NULL)
{
}

Task::~Task()
{
}

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
	
  // Take Information relative to Cost
    slope_values = _slope_values.get();
    locomotion_modes = _locomotion_modes.get();
    cost_data = _cost_data.get();
    risk_distance = _risk_distance.get();
    risk_ratio = _risk_ratio.get();
    reconnect_distance = _reconnect_distance.get();
    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
	
    elevationMatrix = readMatrixFile(_elevationFile.get());

    if (!_traversability_map.connected())
    {
        std::cout << "PLANNER: traversability map input is not connected, a predefined map will be used instead" << std::endl;
        costMatrix = readMatrixFile(_localCostFile.get());
    }

    globalCostMatrix = readMatrixFile(_globalCostFile.get());

	
    planner = new PathPlanning_lib::PathPlanning(cost_data, slope_values, locomotion_modes, risk_distance, reconnect_distance, risk_ratio);

  //pos is the global offset of the Global Map relative to World Frame
  //TODO: set pos externally (and change its name to globalOffset maybe)
    base::Pose2D pos;
    pos.position[0] = 0.0;
    pos.position[1] = 0.0;
	
	
    planner->initGlobalMap(1.0, _local_res, pos, elevationMatrix, globalCostMatrix);  //TODO: 1.0 should be a configurable parameter, maybe global_res


  // Initializing goalWaypoint and wRover
    goalWaypoint.position[0] = 0;
    goalWaypoint.position[1] = 0;
    currentGoal.position[0] = 0;
    currentGoal.position[1] = 0;

    state = BEGINNING;

    std::cout << "PLANNER: initialization completed" << std::endl;

    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

  // Goal Waypoint
    if (_goalWaypoint.read(goalWaypoint) == RTT::NewData)
    {

        if ((goalWaypoint.position[0] != currentGoal.position[0]) ||
           (goalWaypoint.position[1] != currentGoal.position[1]))
        {
            if (planner->setGoal(goalWaypoint))
            {
                state = GLOBAL_PLANNING;
                currentGoal = goalWaypoint;
            }
        }
    }

  // Rover Pose
    if (_pose.read(pose) == RTT::NewData)
    {
        wRover.position = pose.position;
        wRover.heading = pose.getYaw();


        if (state == GLOBAL_PLANNING)
        {
            planner->calculateGlobalPropagation(wRover);
            trajectory.clear();
            trajectory = planner->getNewPath(wRover);
            _trajectory.write(trajectory);
            trajectory2D = trajectory;
            for(uint i = 0; i<trajectory2D.size(); i++)
                trajectory2D[i].position[2] = 0;
            _trajectory2D.write(trajectory2D);
            _global_Total_Cost_map.write(planner->getGlobalTotalCostMap());
            _global_Cost_map.write(planner->getGlobalCostMap());
            state = PATH_COMPUTED;
        }

        if ((state == PATH_COMPUTED)||(state == LOCAL_PLANNING))
        {
            LM::LocomotionMode lm;
            std::string loc = planner->getLocomotionMode(wRover);
            if (loc == "DRIVING")
                lm = LM::DRIVING;
            else if (loc == "WHEEL_WALKING")
                lm = LM::WHEEL_WALKING;
            _locomotionMode.write(lm);
            _actual_total_cost.write(planner->getTotalCost(wRover));
        }
    }
	
  // Traversability Map

    if (state == PATH_COMPUTED)
    {
        if (!_traversability_map.connected())
        {
            if (planner->computeLocalPlanning(wRover, costMatrix, _local_res, trajectory, _keep_old_waypoints))
            {
                _trajectory.write(trajectory);
                trajectory2D = trajectory;
                for(uint i = 0; i<trajectory2D.size(); i++)
                    trajectory2D[i].position[2] = 0;
                _trajectory2D.write(trajectory2D);
            }
        }
        else if (_traversability_map.read(traversability_map) == RTT::NewData)
        {
            //std::cout<< "PLANNER: Starting Traversability map reading loop, period loop is set as" << TaskContext::getPeriod() << std::endl;
            if(planner->computeLocalPlanning(wRover, traversability_map, _local_res, trajectory, _keep_old_waypoints))
            {
                _trajectory.write(trajectory);
                trajectory2D = trajectory;
                for(uint i = 0; i<trajectory2D.size(); i++)
                    trajectory2D[i].position[2] = 0;
                _trajectory2D.write(trajectory2D);
            }
            _local_Risk_map.write(planner->getLocalRiskMap(wRover));
            _local_Propagation_map.write(planner->getLocalPropagationMap(wRover));
            _finished_planning.write(true);
            //std::cout<< "PLANNER: Finishing Traversability map reading loop, period loop is set as" << TaskContext::getPeriod() << std::endl;
        }
    }
}


std::vector< std::vector<double> > Task::readMatrixFile(std::string map_file)
{
    std::cout<< "PLANNER: Reading map " << map_file << std::endl;
    std::vector< std::vector<double> > mapMatrix;
    std::string line;
    std::ifstream eFile(map_file.c_str(), std::ios::in);
    double Nrow = 0, Ncol = 0;
    std::vector <double> row;

    if( eFile.is_open() ){
        while ( std::getline(eFile, line) ){
            std::stringstream ss(line);
            std::string cell;
           while( std::getline(ss, cell, ' ') ){
                double val;
                std::stringstream numericValue(cell);
                numericValue >> val;
                row.push_back(val);
                Ncol++;
            }
            mapMatrix.push_back(row);
            row.clear();
            Nrow++;
        }
    eFile.close();

    Ncol /= Nrow;
        std::cout << "PLANNER: Cost map of " << Ncol
                  << " x "          << Nrow << " loaded." << std::endl;
    } else {
        std::cout << "PLANNER: Problem opening the file" << std::endl;
        return mapMatrix;
    }
    return mapMatrix;
}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
