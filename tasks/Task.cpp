/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <envire/core/Environment.hpp>
#include <envire/maps/TraversabilityGrid.hpp>
#include <envire/operators/SimpleTraversability.hpp>
#include <orocos/envire/Orocos.hpp>

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

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    mTraversabilityMapStatus = RTT::NoData;
    mEnv = new envire::Environment();
    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    readTerrainFile(_soilsFile.get(), costTable);
    elevationMatrix = readMatrixFile(_elevationFile.get());
    costMatrix = readMatrixFile(_costFile.get());


    globalCostMatrix = readMatrixFile(_globalCostFile.get());

    planner = new PathPlanning_lib::PathPlanning(costTable);

    base::Pose2D pos;
    pos.position[0] = 0.0;
    pos.position[1] = 0.0;
    planner->initGlobalMap(1.0, 9, pos, elevationMatrix, globalCostMatrix);
    //globalMap = new PathPlanning_lib::NodeMap(1.0, pos, elevationMatrix, globalCostMatrix,OPEN);
    //map = new PathPlanning_lib::NodeMap(_local_res, pos, costMatrix, costMatrix, HIDDEN);
    //globalMap->makeNeighbourhood();
    //map->hidAll();
    /*std::cout<< "NodeMap created: " << std::endl;
    std::cout<< " - Scale: " << map->scale << std::endl;
    std::cout<< " - Reference Frame: (" << map->globalOriginPose.position [0] << "," << map->globalOriginPose.position [1] << ")"  << std::endl;
    std::cout<< " - Size: " << map->nodeMatrix[0].size() << "x" << map->nodeMatrix.size() << " Nodes" << std::endl;
    std::cout<< " - Debug W: " << map->nodeMatrix[20][20]->work << std::endl;*/
    state = FIRST_GOAL;
    halfTrajectory = false;
    newVisibleArea = false;
    calculatedGlobalWork = false;
    firstIteration = true;
    isArriving = false;
    isClose = false;
    alpha = 0;

    ptu_joints_commands_out.resize(2);
    ptu_joints_commands_out.names[0] = "MAST_PAN";
    ptu_joints_commands_out.names[1] = "MAST_TILT";
    /*ptu_joints_commands_out[0].position = 1.00;
    ptu_joints_commands_out[1].position = 0.00;
    ptu_joints_commands_out[0].speed = base::NaN<float>();
    ptu_joints_commands_out[1].speed = base::NaN<float>();
    _ptu_commands_out.write(ptu_joints_commands_out);*/

    //state = DEBUGGING;
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    if (_goalWaypoint.read(goalWaypoint) == RTT::NewData)
    {
        if (state == FIRST_GOAL)
        {
            state = FIRST_POSE;
            std::cout << "Goal at position (" << goalWaypoint.position[0] <<
                         "," << goalWaypoint.position[1] << "," <<
                         goalWaypoint.position[2] << ") with Heading " <<
                         goalWaypoint.heading << " rad" << std::endl;
            currentGoal = goalWaypoint;
            planner->setGoal(currentGoal);
        }
        if ((state == WAITING) &&
           ((goalWaypoint.position[0] != currentGoal.position[0]) ||
           (goalWaypoint.position[1] != currentGoal.position[1])))
        {
            state = FINDING_PATH;
            std::cout << "Goal at position (" << goalWaypoint.position[0] <<
                         "," << goalWaypoint.position[1] << "," <<
                         goalWaypoint.position[2] << ") with Heading " <<
                         goalWaypoint.heading << " rad" << std::endl;
            currentGoal = goalWaypoint;
            planner->setGoal(currentGoal);
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
        }
        if (sqrt(pow((wRover.position[0] - goalWaypoint.position[0]),2) +
                    pow((wRover.position[1] - goalWaypoint.position[1]),2)) < 0.3)
            isClose = true;

    }

    if((_power_update.read(power_update) == RTT::NewData)&&(calculatedGlobalWork))
    {
        /*if(globalMap->updateNodePower(power_update, wRover, _invert_power))
        {
            std::cout<< "Global Work Map is replanning" << std::endl;
            globalPlanner->fastMarching(goalWaypoint,globalMap);
            map->resetHorizonNodes(globalMap);
            state = FINDING_PATH;
        }*/
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
            planner->calculateGlobalPropagation();

            globalWork = planner->getEnvireGlobalPropagation();
            //globalState = globalMap->getGlobalEnvireState();
            envire::Environment* globalEnv = new envire::Environment();
            globalEnv->attachItem(globalWork);
            //globalEnv->attachItem(globalState);
            envire::OrocosEmitter emitter_global(globalEnv, _global_map);
            emitter_global.setTime(base::Time::now());
            emitter_global.flush();
            std::cout<< "PLANNER: global map as envire map sent" << std::endl;
            calculatedGlobalWork = true;
            firstIteration = true;
            planner->loadLocalArea(wRover);
            newVisibleArea = planner->simUpdateVisibility(wRover, costMatrix, _local_res, TRUE, alpha);
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
            //DEBUGGING

        }
        planner->loadLocalArea(wRover);
        newVisibleArea = planner->simUpdateVisibility(wRover, costMatrix, _local_res, FALSE, alpha);
        planner->calculateLocalPropagation(goalWaypoint,wRover);
        isArriving = planner->getPath(wRover, 0.5, trajectory);
        _trajectory.write(trajectory);

        alpha = atan2(trajectory.back().position[1]-wRover.position[1], trajectory.back().position[0]-wRover.position[0]);
        ptu_joints_commands_out[0].position = alpha - wRover.heading;
        ptu_joints_commands_out[1].position = 0.00;
        ptu_joints_commands_out[0].speed = base::NaN<float>();
        ptu_joints_commands_out[1].speed = base::NaN<float>();
        _ptu_commands_out.write(ptu_joints_commands_out);

        if(newVisibleArea) //TODO: optimize the visualization of the workmap
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
        }

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
        /*
        newVisibleArea = map->updateVisibility(wRover,globalMap,false);
        std::cout<< "New area is visible -> Replanning" << std::endl;

            halfTrajectory = false;
            firstIteration = false;
            localPlanner->fastMarching(goalWaypoint,map,globalMap,wRover);
            if(newVisibleArea) //TODO: optimize the visualization of the workmap
            {
                workGrid = map->getEnvirePropagation(wRover, _crop_local, _work_scale);
                stateGrid = map->getLocalEnvireState(wRover, _crop_local);
                envire::Environment* localEnv = new envire::Environment();
                localEnv->attachItem(workGrid);
                localEnv->attachItem(stateGrid);
                envire::OrocosEmitter emitter_tmp(localEnv, _local_map);
                emitter_tmp.setTime(base::Time::now());
                emitter_tmp.flush();
            }
            newVisibleArea = false;

            isArriving = localPlanner->getPath(map, 0.5, trajectory);
            std::cout<< "Trajectory has " << trajectory.size() << " Waypoints" << std::endl;
            _trajectory.write(trajectory);

            if((isArriving)&&(isClose))
            {
                state = CLOSE_TO_GOAL;
                std::cout<< "Rover is close to the goal, no replanning" << std::endl;
            }
            else
            {
                state = WAITING;
                std::cout<< "Planner is waiting" << std::endl;
            }*/

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
