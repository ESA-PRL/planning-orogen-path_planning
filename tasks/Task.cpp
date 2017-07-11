/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <envire/core/Environment.hpp>
#include <envire/maps/TraversabilityGrid.hpp>
#include <envire/operators/SimpleTraversability.hpp>
#include <orocos/envire/Orocos.hpp>

using namespace path_planning;
using namespace Eigen;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
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
    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    elevationMatrix = readMatrixFile(_elevationFile.get());
    costMatrix = readMatrixFile(_costFile.get());
    riskMatrix = readMatrixFile(_riskFile.get());
    soilList = readTerrainFile(_soilsFile.get());

    planner.initTerrainList(soilList);
    
    double size = 0.05;
    base::Pose2D pos;
    pos.position[0] = 0.0;
    pos.position[1] = 0.0;
    map = new PathPlanning_lib::NodeMap(size, pos, elevationMatrix, costMatrix, riskMatrix);

    std::cout<< "NodeMap created: " << std::endl;
    std::cout<< " - Scale: " << map->nodeWidth << std::endl;
    std::cout<< " - Reference Frame: (" << map->globalOriginPose.position [0] << "," << map->globalOriginPose.position [1] << ")"  << std::endl;
    std::cout<< " - Size: " << map->nodeMatrix[0].size() << "x" << map->nodeMatrix.size() << " Nodes" << std::endl;
    std::cout<< " - Debug W: " << map->nodeMatrix[20][20]->work << std::endl;
    state = WAITING_GOAL;
    //state = DEBUGGING;
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    if (state == DEBUGGING)
    {
        wRover.position[0] = 29;
	wRover.position[1] = 160;
	wRover.position[2] = 0;
	goalWaypoint.position[0] = 160;
	goalWaypoint.position[1] = 110;
	goalWaypoint.position[2] = 0;

        planner.fieldDStar(wRover, goalWaypoint,trajectory,locVector, map);
        state = END;
    }

    if (_goalWaypoint.read(goalWaypoint) == RTT::NewData)
    {
        if (state == WAITING_GOAL)
        {
            state = WAITING_POSE;
            std::cout<< "Goal at position (" << goalWaypoint.position[0] << "," << goalWaypoint.position[1] << "," << goalWaypoint.position[2] << ") with Heading "
                 << goalWaypoint.heading << " rad" << std::endl;
        }
    }

    if (_pose.read(pose) == RTT::NewData)
    {
        if (state == WAITING_POSE)
        {
            wRover.position = pose.position;
            wRover.heading = pose.getYaw();
            std::cout<< "Rover at position (" << wRover.position[0] << "," << wRover.position[1] << "," << wRover.position[2] << ") with Heading "
                     << wRover.heading << " rad" << std::endl;
            state = FINDING_PATH;
        }
    }

    if (state == FINDING_PATH)
    {
	/*wRover.position[0] = wRover.position[0] * 20;
	wRover.position[1] = wRover.position[1] * 20;
	wRover.position[2] = wRover.position[2];
	goalWaypoint.position[0] = goalWaypoint.position[0] * 20;
	goalWaypoint.position[1] = goalWaypoint.position[1] * 20;
	goalWaypoint.position[2] = goalWaypoint.position[2];*/

        planner.fastMarching(wRover,goalWaypoint,trajectory,locVector, map);
        //planner.fieldDStar(wRover, goalWaypoint,trajectory,locVector, map);
        std::cout<< "Trajectory has " << trajectory.size() << " Waypoints" << std::endl;
        for (unsigned int i = 0; i<trajectory.size(); i++)
	{
	    trajectory[i].position[0] = trajectory[i].position[0]/20;
	    trajectory[i].position[1] = trajectory[i].position[1]/20;
	    std::cout << "Waypoint " << i << " -> Pos: (" << trajectory[i].position[0] << "," << trajectory[i].position[1] << ") Height: " << trajectory[i].position[2]
                      << " Heading: " << trajectory[i].heading << " Loc: " << locVector[i] << std::endl;
	}
        _trajectory.write(trajectory);
        _locomotionVector.write(locVector);
        envire::Environment* envireWorkMap = matrix2envire(map);
        envire::OrocosEmitter emitter_tmp(envireWorkMap, _work_map);
        emitter_tmp.setTime(base::Time::now());
        emitter_tmp.flush();
        state = END;
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
        std::cout << "Cost map of " << Nrow
                  << " x "          << Ncol << " loaded." << std::endl;
    } else {
        std::cout << "Problem opening the file" << std::endl;
        return mapMatrix;
    }
    return mapMatrix;
}

std::vector< std::vector<double> > Task::readTerrainFile(std::string terrain_file)
{
    std::cout<< "Map being loaded from: " << terrain_file << ", correct?" << std::endl;
    std::vector< std::vector<double> > terrainList;
    std::string line;
    std::ifstream eFile(terrain_file.c_str(), std::ios::in);
    uint numTerrains = 0, numProperties = 0;
    std::vector <double> row;

    if( eFile.is_open() )
    {
        while ( std::getline(eFile, line) ){
            std::stringstream ss(line);
            std::string cell;
            numProperties = 0;
            while( std::getline(ss, cell, ' ') ){
                double val;
                std::stringstream numericValue(cell);
                numericValue >> val;
                row.push_back(val);
                numProperties++;
            }
            terrainList.push_back(row);
            row.clear();
            numTerrains++;
        }
        eFile.close();
        std::cout << "Terrains detected: " << numTerrains << std::endl;
        std::cout << "Properties detected: " << numProperties << std::endl;
    }
    else
    {
        std::cout << "Problem opening the file" << std::endl;
        return terrainList;
    }
    return terrainList;
}

envire::Environment* Task::matrix2envire(PathPlanning_lib::NodeMap * map)
{
    /*std::cout << "Im here" << std::endl;
    std::vector< std::vector<double> > workMatrix;
    for (uint j = 0; j < map->nodeMatrix[0].size(); j++)
    {
        for (uint i = 0; i < map->nodeMatrix.size(); i++)
        {
            workMatrix[j][i] = map->nodeMatrix[j][i]->work;
        }
    }*/

    std::cout << "Creating Environment" << std::endl;
    envire::Environment* workEnv = new envire::Environment();

    envire::TraversabilityGrid* workGrid = new envire::TraversabilityGrid(map->nodeMatrix.size(), map->nodeMatrix[0].size(), 0.05, 0.05);


    workEnv->attachItem(workGrid);

    envire::TraversabilityGrid::ArrayType& workData = workGrid->getGridData(envire::TraversabilityGrid::TRAVERSABILITY);

    //boost::shared_ptr<envire::TraversabilityGrid::ArrayType> mpWorkData;

    //mpWorkData = boost::shared_ptr<envire::TraversabilityGrid::ArrayType>(&workData, NullDeleter());

    for (uint j = 0; j < map->nodeMatrix[0].size(); j++)
    {
        for (uint i = 0; i < map->nodeMatrix.size(); i++)
        {
            workGrid->setTraversability(map->nodeMatrix[j][i]->work, i, j);
        }
    }


    return workEnv;
}


void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
    delete map;
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
