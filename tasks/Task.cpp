/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"


using namespace path_planning;
using namespace Eigen;
namespace LM = locomotion_switcher;

/*******************************************************************************
** TASK CONSTRUCTOR **
*******************************************************************************/
Task::Task(std::string const& name) : TaskBase(name){}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}


/*******************************************************************************
** TASK DESTRUCTOR **
*******************************************************************************/
Task::~Task() {}


/*******************************************************************************
** CONFIGURE HOOK **
*******************************************************************************/
bool Task::configureHook()
{
    if (!TaskBase::configureHook()) return false;

  // Get Configurable Parameter Values
    slope_values = _slope_values.get();
    global_offset = _global_offset.get();
    locomotion_modes = _locomotion_modes.get();
    cost_data = _cost_data.get();
    risk_distance = _risk_distance.get();
    risk_ratio = _risk_ratio.get();
    reconnect_distance = _reconnect_distance.get();
    return true;
}


/*******************************************************************************
** START HOOK **
*******************************************************************************/
bool Task::startHook()
{
    if (!TaskBase::startHook())
        return false;

  // TODO: enable possibility to also define cost directly using a cost matrix
    elevationMatrix = readMatrixFile(_elevationFile.get());
    terrain_matrix = readMatrixFile(_globalCostFile.get());
    uint map_size_X = elevationMatrix[0].size();
    uint map_size_Y = elevationMatrix.size();

  // The repairing criteria is chosen
    if (_keep_old_waypoints)
        input_approach = PathPlanning_lib::CONSERVATIVE;
    else
        input_approach = PathPlanning_lib::SWEEPING;

  // We initialize the planner introducing the relevant variables for local
  // repairings, in case no local planning is expected, just put random values
  // - risk_distance: max distance from obstacles considered risky
  // - reconnect_distance: in CONSERVATIVE approach, it delimits the position
  //   of the waypoint to reconnect, placed further than this distance to the
  //   risky area
  // - risk_ratio: gradient of the expanded risk, the higher the more pronounced
  //   are the repaired paths
  // - input_approach: the approach
    planner = new PathPlanning_lib::DyMuPathPlanner(risk_distance,
        reconnect_distance, risk_ratio, input_approach);

  // The global layer is here initialized. It is a regular grid formed by global
  // nodes, each of them covering a square portion of the map.
    planner->initGlobalLayer(_global_res, _local_res, map_size_X, map_size_Y,
                             global_offset);

  // Cost values must be assigned to each of the global nodes. In this case, we
  // make use of cost based on locomotion according to slope (computed from the
  // elevation) and type of terrain
  // Btw, do not forget here the terrain 0 is considered as obstacle (non-tra-
  // versable)
    if (!planner->computeCostMap(cost_data, slope_values, locomotion_modes,
                                 elevationMatrix, terrain_matrix))
        LOG_ERROR_S << "Cost Map Computation failed";

  // The goal is set to the infinity (a bit improbable to reach there...)
    goalWaypoint.position[0] = std::numeric_limits<double>::infinity();
    goalWaypoint.position[1] = std::numeric_limits<double>::infinity();
    currentGoal.position[0] = std::numeric_limits<double>::infinity();
    currentGoal.position[1] = std::numeric_limits<double>::infinity();


  // Initializing output file to write time values of local planning operations
    if (_write_results)
        localTimeFile.open("LocalTimeValues.txt");

    num_globalpp_executions = 0;

  // After everything is set up, the planner is ready for receiving the goal
    state(WAITING_FOR_GOAL);
    return true;
}


/*******************************************************************************
** UPDATE HOOK **
*******************************************************************************/
void Task::updateHook()
{
    TaskBase::updateHook();

  // A new Goal triggers a new path planning operation (in case its pose
  //  is valid)
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

  // Whenever a new pose is received, a path is computed if the goal has
  // previously changed
    if (_pose.read(pose) == RTT::NewData)
    {
        wRover.position = pose.position;
        wRover.heading = pose.getYaw();

        if (state() == WAITING_FOR_POSE)
        {
            if(planner->computeTotalCostMap(wRover))
            {
                trajectory.clear();
                std::cout << "New path incoming" << std::endl;
                trajectory = planner->getPath(wRover);
                _trajectory.write(trajectory);
                trajectory2D = trajectory;
                for (uint i = 0; i < trajectory2D.size(); i++)
                {
                    trajectory2D[i].position[2] = 0;
                }
                _trajectory2D.write(trajectory2D);
                if (_write_results)
                {
                    writeGlobalResults();
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

  // If the path is already created, a map of traversability may trigger a local
  // path repairing
    if (state() == PATH_COMPUTED)
    {
      // This is used for simulations
        if (_set_random_travmap.read(set_travmap) == RTT::NewData)
        {
            if (set_travmap)
            {
                base::samples::frame::Frame random_trav_map;
                double width = 100;
                double height = 100;
                random_trav_map.init(width, height);
                double obstacle_center_x = width/2.0 + 30.0*cos(wRover.heading);
                double obstacle_center_y = height/2.0 - 30.0*sin(wRover.heading);
                double obstacle_radius = 10.0;
                for (uint j = 0; j < height; j++)
                    for (uint i = 0; i < width; i++)
                        if (sqrt(pow((float)i-obstacle_center_x,2)+pow((float)j-obstacle_center_y,2)) < obstacle_radius)
                            random_trav_map.image[random_trav_map.getRowSize()*j
                            +i*random_trav_map.getPixelSize()] = 1;
                        else
                            random_trav_map.image[random_trav_map.getRowSize()*j
                            +i*random_trav_map.getPixelSize()] = 0;
                if (planner->computeLocalPlanning(
                        wRover, random_trav_map, _local_res, trajectory,
                        local_computation_time))
                {
                    _trajectory.write(trajectory);
                    trajectory2D = trajectory;
                    for (uint i = 0; i < trajectory2D.size(); i++)
                        trajectory2D[i].position[2] = 0;
                    _trajectory2D.write(trajectory2D);
                    if (_write_results)
                    {
                        writeLocalResults();
                        writeGlobalResults();
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
                    local_computation_time))
            {
                _trajectory.write(trajectory);
                trajectory2D = trajectory;
                for (uint i = 0; i < trajectory2D.size(); i++)
                    trajectory2D[i].position[2] = 0;
                _trajectory2D.write(trajectory2D);
                if (_write_results)
                    {
                        writeLocalResults();
                        writeGlobalResults();
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

void Task::writeGlobalResults()
{
    total_cost_matrix = planner->getTotalCostMatrix();
    global_cost_matrix = planner->getGlobalCostMatrix();
    hazard_density_matrix = planner->getHazardDensityMatrix();
    trafficability_matrix = planner->getTrafficabilityMatrix();
    std::string total_cost_filename =
    std::string("TotalCostMap_") +
    std::to_string(num_globalpp_executions) +
     ".txt";
    std::string global_cost_filename =
                                           std::string("GlobalCostMap_") +
                                         std::to_string(num_globalpp_executions)
                                         + ".txt";
    std::string hazard_density_filename =
                                           std::string("HazardDensityMap_") +
                                         std::to_string(num_globalpp_executions)
                                         + ".txt";
    std::string trafficability_filename =
                                           std::string("TrafficabilityMap_") +
                                         std::to_string(num_globalpp_executions)
                                         + ".txt";
    std::string path_filename = std::string("Path_") +
                                         std::to_string(num_globalpp_executions)
                                         + ".txt";
    total_cost_file.open(total_cost_filename);
    global_cost_file.open(global_cost_filename);
    hazard_density_file.open(hazard_density_filename);
    trafficability_file.open(trafficability_filename);
    path_file.open(path_filename);
    for (uint j = 0; j < total_cost_matrix.size(); j++)
    {
        for (uint i = 0; i < total_cost_matrix[0].size()-1; i++)
        {
            total_cost_file << total_cost_matrix[j][i] << " ";
            global_cost_file << global_cost_matrix[j][i] << " ";
            hazard_density_file << hazard_density_matrix[j][i] << " ";
            trafficability_file << trafficability_matrix[j][i] << " ";
        }
        total_cost_file << total_cost_matrix[j][total_cost_matrix[0].size()-1];
        global_cost_file << global_cost_matrix[j][total_cost_matrix[0].size()-1];
        hazard_density_file << hazard_density_matrix[j][total_cost_matrix[0].size()-1];
        trafficability_file << trafficability_matrix[j][total_cost_matrix[0].size()-1];
        total_cost_file << "\n";
        global_cost_file << "\n";
        hazard_density_file << "\n";
        trafficability_file << "\n";
    }
    for (uint k = 0; k < trajectory2D.size(); k++)
        path_file << trajectory2D[k].position[0] << " " <<
        trajectory2D[k].position[1] << "\n";
    total_cost_file.close();
    global_cost_file.close();
    hazard_density_file.close();
    trafficability_file.close();
    path_file.close();
    num_globalpp_executions++;
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

void Task::writeLocalResults()
{
    localTimeFile << "Local Propagation Time: " <<
                     local_computation_time.toSeconds() << "\n";
    _local_computation_time.write(local_computation_time);
    risk_matrix = planner->getRiskMatrix(wRover);
    deviation_matrix = planner->getDeviationMatrix(wRover);
    std::string risk_filename =
                           std::string("RiskMap_") +
                         std::to_string(num_globalpp_executions)
                         + ".txt";
    std::string deviation_filename =
                           std::string("DeviationMap_") +
                         std::to_string(num_globalpp_executions)
                         + ".txt";
    risk_file.open(risk_filename);
    deviation_file.open(deviation_filename);
    for (uint j = 0; j < risk_matrix.size(); j++)
    {
        for (uint i = 0; i < risk_matrix[0].size(); i++)
        {
            risk_file << risk_matrix[j][i] << " ";
            deviation_file << deviation_matrix[j][i] << " ";
        }
        deviation_file << "\n";
        risk_file << "\n";
    }
    risk_file.close();
    deviation_file.close();
}

void Task::errorHook() { TaskBase::errorHook(); }
void Task::stopHook()
{
    TaskBase::stopHook();
    localTimeFile.close();
}
void Task::cleanupHook() { TaskBase::cleanupHook(); }
