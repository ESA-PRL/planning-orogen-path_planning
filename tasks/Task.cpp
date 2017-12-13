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
    //mTraversabilityMapStatus = RTT::NoData;
    //mEnv = new envire::Environment();
    slope_values = _slope_values.get();
    locomotion_modes = _locomotion_modes.get();
    cost_data = _cost_data.get();
    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    for(uint i = 0; i<slope_values.size(); i++)
        std::cout << "PLANNER: slope " << i << " is " << slope_values[i] << std::endl;
    readTerrainFile(_soilsFile.get(), costTable);
    elevationMatrix = readMatrixFile(_elevationFile.get());

    if (!_traversability_map.connected())
    {
        std::cout << "PLANNER: traversability map input is not connected, a predefined map will be used instead" << std::endl;
        costMatrix = readMatrixFile(_costFile.get());
    }

    globalCostMatrix = readMatrixFile(_globalCostFile.get());

    planner = new PathPlanning_lib::PathPlanning(costTable, cost_data,slope_values,locomotion_modes);

    base::Pose2D pos;
    pos.position[0] = 0.0;
    pos.position[1] = 0.0;
    planner->initGlobalMap(1.0, _local_res, pos, elevationMatrix, globalCostMatrix);

    state = FIRST_GOAL;
    halfTrajectory = false;
    //newVisibleArea = false;
    pathNeedsRepair = false;
    calculatedGlobalWork = false;
    firstIteration = true;
    isArriving = false;
    isClose = false;

    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    if (_goalWaypoint.read(goalWaypoint) == RTT::NewData)
    {
        if (state == FIRST_GOAL)
        {
            if (planner->setGoal(goalWaypoint))
            {
                state = FIRST_POSE;
                currentGoal = goalWaypoint;
            }
        }
        if ((state == WAITING) &&
           ((goalWaypoint.position[0] != currentGoal.position[0]) ||
           (goalWaypoint.position[1] != currentGoal.position[1])))
        {
            if (planner->setGoal(goalWaypoint))
            {
                state = FINDING_PATH;
                currentGoal = goalWaypoint;
                calculatedGlobalWork = false;
            }
        }
    }

    if (_pose.read(pose) == RTT::NewData)
    {
        wRover.position = pose.position;
        wRover.heading = pose.getYaw();
        if (state == FIRST_POSE)
        {
            state = FINDING_PATH;
            std::cout << "Rover at position (" << wRover.position[0] <<
                         "," << wRover.position[1] << "," <<
                         wRover.position[2] << ") with Heading " <<
                         wRover.heading << " rad" << std::endl;
            currentPos = wRover;
        }
        if ((state == WAITING) &&
           ((sqrt(pow((wRover.position[0] - currentPos.position[0]),2) +
                    pow((wRover.position[1] - currentPos.position[1]),2)) > 0.1)||
            (acos(cos(wRover.heading)*cos(currentPos.heading) + sin(wRover.heading)*sin(currentPos.heading)) > 5*3.14/180)))
        {
            state = FINDING_PATH;
            std::cout << "Rover at position (" << wRover.position[0] <<
                         "," << wRover.position[1] << "," <<
                         wRover.position[2] << ") with Heading " <<
                         wRover.heading << " rad" << std::endl;
            currentPos = wRover;
        }
        if (calculatedGlobalWork)
        {
            LM::LocomotionMode lm;
            std::string loc = planner->getLocomotionMode(wRover);
            if (loc == "DRIVING")
                lm = LM::DRIVING;
            else if (loc == "WHEEL_WALKING")
                lm = LM::WHEEL_WALKING;
            _locomotionMode.write(lm);
            _actual_total_cost.write(planner->getInterpolatedCost(wRover));
        }
        if (sqrt(pow((wRover.position[0] - goalWaypoint.position[0]),2) +
                    pow((wRover.position[1] - goalWaypoint.position[1]),2)) < 0.3)
            isClose = true;
    }

    if(_traversability_map.read(traversability_map) == RTT::NewData)
    {
        if((state == FINDING_PATH)&&(calculatedGlobalWork))
            if(planner->evaluateLocalMap(wRover, traversability_map, _local_res, trajectory))
            {
                _trajectory.write(trajectory);
                trajectory2D = trajectory;
                for(uint i = 0; i<trajectory2D.size(); i++)
                    trajectory2D[i].position[2] = 0;
                _trajectory2D.write(trajectory2D);
            }
    }

    if((_slip_ratio.read(slip_ratio) == RTT::NewData)&&(calculatedGlobalWork))
    {
        /*std::cout<< "PLANNER: received slip value" << std::endl;
        if(globalMap->updateNodeSlip(slip_ratio, wRover))
        {
            std::cout<< "PLANNER: Global Work Map is replanning due to change of slip value" << std::endl;
            globalPlanner->fastMarching(goalWaypoint,globalMap);
            map->resetHorizonNodes(globalMap);
            //FOR DEBUGGING
            for(uint j = 0; j < globalMap->nodeMatrix.size(); j++)
            {
                for(uint i = 0; i < globalMap->nodeMatrix[0].size();i++)
                    std::cout << globalMap->nodeMatrix[j][i]->total_cost << " ";
                std::cout << std::endl;
            }
            state = FINDING_PATH;
            globalWork = globalMap->getEnvirePropagation(wRover,false, _work_scale);
            globalState = globalMap->getGlobalEnvireState();
            envire::Environment* globalEnv = new envire::Environment();
            globalEnv->attachItem(globalWork);
            globalEnv->attachItem(globalState);
            envire::OrocosEmitter emitter_global(globalEnv, _global_map);
            emitter_global.setTime(base::Time::now());
            emitter_global.flush();
        }*/
    }

    if (state == FINDING_PATH)
    {
        if(!calculatedGlobalWork)
        {
            //globalPlanner->fastMarching(goalWaypoint,globalMap);
            planner->calculateGlobalPropagation(wRover);

            /*globalWork = planner->getEnvireGlobalPropagation();
            //globalState = globalMap->getGlobalEnvireState();
            envire::Environment* globalEnv = new envire::Environment();
            globalEnv->attachItem(globalWork);
            //globalEnv->attachItem(globalState);
            envire::OrocosEmitter emitter_global(globalEnv, _global_map);
            emitter_global.setTime(base::Time::now());
            emitter_global.flush();*/
            _global_Total_Cost_map.write(planner->getGlobalTotalCostMap());
            _global_Cost_map.write(planner->getGlobalCostMap());
            std::cout<< "PLANNER: global map as envire map sent" << std::endl;
            calculatedGlobalWork = true;
            firstIteration = true;
            trajectory.clear();
            trajectory = planner->getNewPath(wRover);
            planner->evaluatePath(trajectory); //Evaluate if it passes through already discovered risky areas

            planner->updateLocalMap(wRover);
            pathNeedsRepair = planner->evaluateLocalMap(wRover, costMatrix, _local_res, trajectory);
            _trajectory.write(trajectory);
            trajectory2D = trajectory;
            for(uint i = 0; i<trajectory2D.size(); i++)
                trajectory2D[i].position[2] = 0;
            _trajectory2D.write(trajectory2D);
            //newVisibleArea = false;
        }
        else
        {
            _global_Cost_map.write(planner->getGlobalCostMap());
            _local_Total_Cost_map.write(planner->getLocalTotalCostMap(wRover));
            planner->updateLocalMap(wRover);
            if (!_traversability_map.connected())
                if(planner->evaluateLocalMap(wRover, costMatrix, _local_res, trajectory))
                {
                    _trajectory.write(trajectory);
                    trajectory2D = trajectory;
                    for(uint i = 0; i<trajectory2D.size(); i++)
                        trajectory2D[i].position[2] = 0;
                    _trajectory2D.write(trajectory2D);
                }
            _local_Risk_map.write(planner->getLocalRiskMap(wRover));
        }

        /*localState = planner->getEnvireLocalState(wRover);
        riskGrid = planner->getEnvireRisk(wRover);
        costGrid = planner->getLocalTotalCost(wRover);
        envire::Environment* localEnv = new envire::Environment();
        localEnv->attachItem(localState);
        localEnv->attachItem(costGrid);
        envire::OrocosEmitter emitter_tmp(localEnv, _local_map);
        emitter_tmp.setTime(base::Time::now());
        emitter_tmp.flush();*/


        /*planner->calculateLocalPropagation(goalWaypoint,wRover);
        isArriving = planner->getPath(wRover, 0.5, trajectory);
        _trajectory.write(trajectory);*/

        /*if(newVisibleArea) //TODO: optimize the visualization of the workmap
        {
            localState = planner->getEnvireLocalState(wRover);
            riskGrid = planner->getEnvireRisk(wRover);
            costGrid = planner->getLocalTotalCost(wRover);
            envire::Environment* localEnv = new envire::Environment();
            localEnv->attachItem(localState);
            localEnv->attachItem(costGrid);
            envire::OrocosEmitter emitter_tmp(localEnv, _local_map);
            emitter_tmp.setTime(base::Time::now());
            emitter_tmp.flush();
            newVisibleArea = false;
        }*/

        if((isArriving)&&(isClose))
        {
            state = CLOSE_TO_GOAL;
            std::cout << "PLANNER: Rover is close to the goal, no replanning" << std::endl;
        }
        else
        {
            state = WAITING;
            std::cout<< "PLANNER: Planner waits for updates" << std::endl;
        }
    }
}


std::vector< std::vector<double> > Task::readMatrixFile(std::string map_file)
{
    std::cout<< "Map being loaded from: " << map_file << ", correct?" << std::endl;
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
        std::cout << "Cost map of " << Ncol
                  << " x "          << Nrow << " loaded." << std::endl;
    } else {
        std::cout << "Problem opening the file" << std::endl;
        return mapMatrix;
    }
    return mapMatrix;
}

void Task::readTerrainFile(std::string terrain_file, std::vector< PathPlanning_lib::terrainType* >& table)
{
    std::cout<< "Extracting terrain data from: " << terrain_file << std::endl;
    std::vector< std::vector<double> > terrainList;
    std::string line;
    std::ifstream eFile(terrain_file.c_str(), std::ios::in);
    uint numTerrains = 0;
    std::cout << "TERRAIN - COST - LOCOMOTION"  << std::endl;
    if( eFile.is_open() )
    {
        while ( std::getline(eFile, line) ){
            std::stringstream ss(line);
            std::string cell;
            table.push_back(new PathPlanning_lib::terrainType);
            std::getline(ss, cell, ' ');
            std::stringstream numericValue(cell);
            numericValue >> table.back()->cost;
            std::getline(ss, cell, ' ');
            table.back()->optimalLM = cell;
            numTerrains++;
        }
        eFile.close();
        for (uint i = 0; i<table.size(); i++)
            std::cout << i << " " << table[i]->cost << " "  << table[i]->optimalLM << std::endl;
        std::cout << "Terrains detected: " << numTerrains << std::endl;
    }
    else
    {
        std::cout << "Problem opening the file" << std::endl;
    }
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
