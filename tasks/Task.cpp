/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <envire/core/Environment.hpp>
#include <envire/maps/TraversabilityGrid.hpp>
#include <envire/operators/SimpleTraversability.hpp>
#include <orocos/envire/Orocos.hpp>

using namespace path_planning;
using namespace Eigen;

Task::Task(std::string const& name)
    : TaskBase(name),
      mEnv(NULL),
      localNodeMap(NULL)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine),
      mEnv(NULL),
      localNodeMap(NULL)
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

    elevationMatrix = readMatrixFile(_elevationFile.get());
    costMatrix = readMatrixFile(_costFile.get());
    riskMatrix = readMatrixFile(_riskFile.get());
    soilList = readTerrainFile(_soilsFile.get());

    globalCostMatrix = readMatrixFile(_globalCostFile.get());

    globalPlanner = new PathPlanning_lib::PathPlanning(GLOBAL_PLANNER);
    localPlanner = new PathPlanning_lib::PathPlanning(LOCAL_PLANNER);

    globalPlanner->initTerrainList(soilList);
    localPlanner->initTerrainList(soilList);

    double size = 0.05;
    base::Pose2D pos;
    pos.position[0] = 0.0;
    pos.position[1] = 0.0;
    map = new PathPlanning_lib::NodeMap(size, pos, elevationMatrix, costMatrix);
    globalMap = new PathPlanning_lib::NodeMap(1.0, pos, globalCostMatrix, globalCostMatrix);
    map->hidAll();
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
    current_segment = 0;


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

        globalPlanner->fieldDStar(wRover, goalWaypoint,trajectory,locVector, map);
        state = END;
    }

    /*RTT::FlowStatus ret = receiveEnvireData();

    if (ret == RTT::NewData)
    {
        RTT::log(RTT::Info) <<  "SimplePathglobalPlanner: Received new environment data" << RTT::endlog();
        std::cout << "SimplePathglobalPlanner: Received new environment data" << std::endl;

    }*/

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
           ((wRover.position[0] != currentPos.position[0]) ||
           (wRover.position[1] != currentPos.position[1])))
        {
            state = FINDING_PATH;
            std::cout << "Rover at position (" << wRover.position[0] <<
                         "," << wRover.position[1] << "," <<
                         wRover.position[2] << ") with Heading " <<
                         wRover.heading << " rad" << std::endl;
            currentPos = wRover;
        }
    }

    if (state == FINDING_PATH)
    {
        if(!calculatedGlobalWork)
        {
            globalPlanner->fastMarching(wRover,goalWaypoint,globalMap,NULL);
            std::cout<< "Global Work Map created" << std::endl;
            calculatedGlobalWork = true;
            firstIteration = true;
        }
        newVisibleArea = map->updateVisibility(wRover);
        /*if(newVisibleArea)
            std::cout<< "New area is visible -> Replanning" << std::endl;*/

        _current_segment.read(current_segment);
        if ((2*current_segment) > trajectory.size())
        {
            std::cout<< "Traversed half of the trajectory -> Replanning" << std::endl;
            halfTrajectory = true;
        }

        //if ((newVisibleArea)||(halfTrajectory))
        if ((firstIteration)||(halfTrajectory))
        {
            halfTrajectory = false;
            newVisibleArea = false;
            firstIteration = false;
            localPlanner->fastMarching(wRover,goalWaypoint,map,globalMap);
            //trajectory.clear();
            //locVector.clear();
            isArriving = localPlanner->getPath(map, 0.5, trajectory, locVector, current_segment);
            std::cout<< "Trajectory has " << trajectory.size() << " Waypoints" << std::endl;
            /*for (unsigned int i = 0; i<trajectory.size(); i++)
    	      {
    	          std::cout << "Waypoint " << i << " -> Pos: (" << trajectory[i].position[0] << "," << trajectory[i].position[1] << ") Height: " << trajectory[i].position[2]
                          << " Heading: " << trajectory[i].heading << " Loc: " << locVector[i] << std::endl;
    	      }*/
            std::cout << "Back Waypoint -> Pos: (" << trajectory.back().position[0] << "," << trajectory.back().position[1] << ") Height: " << trajectory.back().position[2]
                      << " Heading: " << trajectory.back().heading << " Loc: " << locVector.back() << std::endl;
            _trajectory.write(trajectory);
            _locomotionVector.write(locVector);



            workGrid = map->getEnvirePropagation();
            stateGrid = map->getEnvireState();
        /*for (uint j = 0; j < map->nodeMatrix[0].size(); j++)
        {
            for (uint i = 0; i < map->nodeMatrix.size(); i++)
            {
                std::cout << "Work of node (" << i
                          << "," << j << ") is "         << workGrid->getGridData()[(double)(i)*scale][(double)(j)*scale] << std::endl;
            }
        }*/
            envire::Environment* workEnv = new envire::Environment();
            workEnv->attachItem(workGrid);
            workEnv->attachItem(stateGrid);
            envire::OrocosEmitter emitter_tmp(workEnv, _work_map);
            emitter_tmp.setTime(base::Time::now());
            emitter_tmp.flush();
            if(isArriving)
            {
                state = CLOSE_TO_GOAL;
                std::cout<< "Rover is close to the goal, no replanning" << std::endl;
            }
            else
            {
                state = WAITING;
                std::cout<< "Planner is waiting" << std::endl;
            }
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

// Extracted from: rock-planning/planning-orogen-simple_path_globalPlanner
RTT::FlowStatus Task::receiveEnvireData()
{
    envire::OrocosEmitter::Ptr binary_event;
    RTT::FlowStatus ret = mTraversabilityMapStatus;
    while(_traversability_map.read(binary_event) == RTT::NewData)
    {
        ret = RTT::NewData;
        mEnv->applyEvents(*binary_event);
    }

    if ((ret == RTT::NoData) || (ret == RTT::OldData))
    {
        return ret;
    }

    // Just to add height informations to the trajectory.
    //extractMLS();

    // Extracts data and adds it to the globalPlanner.
    if(!extractTraversability()) {
        return mTraversabilityMapStatus;
    }

    // Set from NoData to OldData, variable should only be used locally.
    mTraversabilityMapStatus = RTT::OldData;
    return RTT::NewData;
}

bool Task::extractTraversability() {
    std::vector<envire::TraversabilityGrid*> maps = mEnv->getItems<envire::TraversabilityGrid>();

    // Lists all received traversability maps.
    std::stringstream ss;
    if(maps.size()) {
        ss << "SimplePathglobalPlanner: Received traversability map(s): " << std::endl;

        std::string trav_map_id;
        std::vector<envire::TraversabilityGrid*>::iterator it = maps.begin();
        for(int i=0; it != maps.end(); ++it, ++i)
        {
            ss << i << ": " << (*it)->getUniqueId() << std::endl;
        }
        RTT::log(RTT::Info) << ss.str() << RTT::endlog();
    } else {
        RTT::log(RTT::Warning) << "SimplePathglobalPlanner: Environment does not contain any traversability grids" << RTT::endlog();
        return false;
    }

    if(!maps.size() || maps.size() > 1) {
        throw std::runtime_error("simple_path_globalPlanner::Task:Environment contains more than one TraversabilityGrid");
    }

    envire::TraversabilityGrid* traversability = *(maps.begin());
    if(!traversability->getFrameNode())
        throw std::runtime_error("simple_path_globalPlanner::Task:Error, grid has no framenode");

    RTT::log(RTT::Info) << "SimplePathglobalPlanner: Traversability map " << traversability->getUniqueId() << " extracted" << RTT::endlog();

    // Adds the trav map to the globalPlanner.
    //mglobalPlanner->updateTraversabilityMap(traversability);

    std::cout << "Size of local map is: " << traversability->getCellSizeX() << "x" << traversability->getCellSizeY() << std::endl;

    uint aa = 10, bb = 10;

    std::vector<envire::TraversabilityClass> cc = traversability->getTraversabilityClasses();

    uint dd = traversability->getGridData()[bb][aa];

    std::cout << "Number of classes is: " << dd << std::endl;

    /*std::vector< std::vector<double> > localElevationMatrix;
    std::vector< std::vector<double> > localCostMatrix;
    std::vector< std::vector<double> > localRiskMatrix;

    std::vector<double> row;

    localElevationMatrix.resize(getCellSizeY(),std::vector<double>);
    for (uint i = 0, i<traversability->getCellSizeX(),i++)
        localElevationMatrix[i].resize(getCellSizeX(),double);

    for (uint j = 0, j<traversability->getCellSizeY(),j++)
    {
        for (uint i = 0, i<traversability->getCellSizeX(),i++)
        {

        }
    }

    double size = 0.05;
    base::Pose2D pos;
    pos.position[0] = 0.0;
    pos.position[1] = 0.0;*/

    if (localNodeMap == NULL)
        localNodeMap = new PathPlanning_lib::NodeMap(traversability);
    else
        localNodeMap->updateNodeMap(traversability);
    //map->createLocalNodeMap(traversability);

    return true;
}
