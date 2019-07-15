/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"


using namespace path_planning;
using namespace Eigen;
namespace LM = locomotion_switcher;

Task::Task(std::string const& name) : TaskBase(name), mEnv(NULL) {}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine), mEnv(NULL)
{
}

Task::~Task() {}

bool Task::configureHook()
{
    if (!TaskBase::configureHook()) return false;

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
    if (!TaskBase::startHook())
    {
        return false;
    }

    elevationMatrix = readMatrixFile(_elevationFile.get());

    if (!_traversability_map.connected())
    {
        LOG_DEBUG_S
            << "traversability map input is not connected, a predefined map will be used instead";
        costMatrix = readMatrixFile(_localCostFile.get());
    }

    terrain_matrix = readMatrixFile(_globalCostFile.get());

    planner = new PathPlanning_lib::DyMuPathPlanner(
        cost_data, slope_values, locomotion_modes, risk_distance,
        reconnect_distance, risk_ratio);

    // pos is the global offset of the Global Map relative to World Frame
    // TODO: set pos externally (and change its name to globalOffset maybe)
    base::Pose2D offset;
    offset.position[0] = 0.0;
    offset.position[1] = 0.0;

    uint map_size_X = elevationMatrix[0].size();
    uint map_size_Y = elevationMatrix.size();

    std::cout << "PLANNER_TASK: size is " << map_size_X << " x " <<
        map_size_Y << std::endl;
    planner->initGlobalLayer(_global_res, _local_res, map_size_X, map_size_Y,
                             offset);

    if (!planner->computeCostMap(elevationMatrix, terrain_matrix,true))//TODO: Make this true configurable
        std::cout << "PLANNER_TASK: size is " << map_size_X << " x " <<
            map_size_Y << std::endl;


    // Initializing goalWaypoint and wRover
    goalWaypoint.position[0] = 0;
    goalWaypoint.position[1] = 0;
    currentGoal.position[0] = 0;
    currentGoal.position[1] = 0;

    state(WAITING_FOR_GOAL);
    // Initializing output file to write time values of local planning operations
    if (_write_results)
        localTimeFile.open("LocalTimeValues.txt");

    LOG_DEBUG_S << "initialization completed";

    num_globalpp_executions = 0;

    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    // Goal Waypoint
    if (_goalWaypoint.read(goalWaypoint) == RTT::NewData)
    {
        if ((goalWaypoint.position[0] != currentGoal.position[0])
            || (goalWaypoint.position[1] != currentGoal.position[1]))
        {
            if (planner->setGoal(goalWaypoint))
            {
                state(WAITING_FOR_POSE);
                currentGoal = goalWaypoint;
            }
            else
            {
                state(NON_VALID_GOAL);
            }
        }
    }

    // Rover Pose
    if (_pose.read(pose) == RTT::NewData)
    {
        wRover.position = pose.position;
        wRover.heading = pose.getYaw();

        if (state() == WAITING_FOR_POSE)
        {
            if(planner->computeTotalCostMap(wRover))
            {
                trajectory.clear();
                trajectory = planner->getNewPath(wRover);
                _trajectory.write(trajectory);
                trajectory2D = trajectory;
                for (uint i = 0; i < trajectory2D.size(); i++)
                {
                    trajectory2D[i].position[2] = 0;
                }
                _trajectory2D.write(trajectory2D);
                if (_write_results)
                {
                    total_cost_matrix = planner->getTotalCostMatrix();
                    global_cost_matrix = planner->getGlobalCostMatrix();
                    std::string total_cost_filename =
                                           std::string("TotalCostMap_") +
                                         std::to_string(num_globalpp_executions)
                                         + ".txt";
                    std::string global_cost_filename =
                                           std::string("GlobalCostMap_") +
                                         std::to_string(num_globalpp_executions)
                                         + ".txt";
                    std::string path_filename = std::string("Path_") +
                                         std::to_string(num_globalpp_executions)
                                         + ".txt";
                    total_cost_file.open(total_cost_filename);
                    global_cost_file.open(global_cost_filename);
                    path_file.open(path_filename);
                    for (uint j = 0; j < total_cost_matrix.size(); j++)
                    {
                        for (uint i = 0; i < total_cost_matrix[0].size(); i++)
                        {
                            total_cost_file << total_cost_matrix[j][i] << " ";
                            global_cost_file << global_cost_matrix[j][i] << " ";
                        }
                        total_cost_file << "\n";
                        global_cost_file << "\n";
                    }
                    for (uint k = 0; k < trajectory2D.size(); k++)
                        path_file << trajectory2D[k].position[0] << " " <<
                                     trajectory2D[k].position[1] << "\n";
                    total_cost_file.close();
                    global_cost_file.close();
                    path_file.close();
                }
                state(PATH_COMPUTED);
            }
            else
            {
                state(UNREACHABLE_GOAL);
            }
        }

        if (state() == PATH_COMPUTED)
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

    if (state() == PATH_COMPUTED)
    {
        if (_set_random_travmap.read(set_travmap) == RTT::NewData)
        {
            if (set_travmap)
            {
                base::samples::frame::Frame random_trav_map;
                uint width = 100;
                uint height = 100;
                random_trav_map.init(width, height);
                for (uint j = 0; j < height; j++)
                {
                    for (uint i = 0; i < width; i++)
                    {
                        if (i < 10)
                        {
                            random_trav_map.image[random_trav_map.getRowSize()*j
                            +i*random_trav_map.getPixelSize()] = 1;
                        }
                        else
                        {
                            random_trav_map.image[random_trav_map.getRowSize()*j
                            +i*random_trav_map.getPixelSize()] = 0;
                        }
                    }
                    std::cout << std::endl;
                }
                if (planner->computeLocalPlanning(
                        wRover, random_trav_map, _local_res, trajectory,
                        _keep_old_waypoints, local_computation_time))
                {
                    _trajectory.write(trajectory);
                    trajectory2D = trajectory;
                    for (uint i = 0; i < trajectory2D.size(); i++)
                        trajectory2D[i].position[2] = 0;
                    _trajectory2D.write(trajectory2D);
                    if (_write_results)
                    {
                        localTimeFile << "Local Propagation Time: " <<
                                         local_computation_time.toSeconds() << "\n";
                        _local_computation_time.write(local_computation_time);
                    }
                }
                //_local_Risk_map.write(planner->getLocalRiskMap(wRover));
                //_local_Propagation_map.write(planner->getLocalPropagationMap(wRover));
                _finished_planning.write(true);
            }
        }
        if (_traversability_map.read(traversability_map) == RTT::NewData)
        {
            if (planner->computeLocalPlanning(
                    wRover, traversability_map, _local_res, trajectory,
                    _keep_old_waypoints, local_computation_time))
            {
                _trajectory.write(trajectory);
                trajectory2D = trajectory;
                for (uint i = 0; i < trajectory2D.size(); i++)
                    trajectory2D[i].position[2] = 0;
                _trajectory2D.write(trajectory2D);
                if (_write_results)
                {
                    localTimeFile << "Local Propagation Time: " <<
                                     local_computation_time.toSeconds() << "\n";
                    _local_computation_time.write(local_computation_time);
                }
            }
            //_local_Risk_map.write(planner->getLocalRiskMap(wRover));
            //_local_Propagation_map.write(planner->getLocalPropagationMap(wRover));
            _finished_planning.write(true);
            // LOG_DEBUG_S << "Finishing Traversability map reading loop, period loop is set as" <<
            // TaskContext::getPeriod();
        }
    }
}

std::vector<std::vector<double>> Task::readMatrixFile(std::string map_file)
{
    LOG_DEBUG_S << "Reading map " << map_file;
    std::vector<std::vector<double>> mapMatrix;
    std::string line;
    std::ifstream eFile(map_file.c_str(), std::ios::in);
    double Nrow = 0, Ncol = 0;
    std::vector<double> row;

    if (eFile.is_open())
    {
        while (std::getline(eFile, line))
        {
            std::stringstream ss(line);
            std::string cell;
            while (std::getline(ss, cell, ' '))
            {
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
        LOG_DEBUG_S << "PLANNER: Cost map of " << Ncol << " x " << Nrow << " loaded.";
    }
    else
    {
        LOG_DEBUG_S << "PLANNER: Problem opening the file";
        return mapMatrix;
    }
    return mapMatrix;
}

void Task::errorHook() { TaskBase::errorHook(); }
void Task::stopHook()
{
    TaskBase::stopHook();
    localTimeFile.close();
}
void Task::cleanupHook() { TaskBase::cleanupHook(); }
