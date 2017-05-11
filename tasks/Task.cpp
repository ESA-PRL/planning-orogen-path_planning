/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

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
    elevationMatrix = readMatrixFile("../terrainData/crater/elevation_map.txt", Nraw, Ncol);
    frictionMatrix = readMatrixFile("../terrainData/crater/crater_costMap.txt", Nraw, Ncol);
    slipMatrix = readMatrixFile("../terrainData/crater/slip_map.txt", Nraw, Ncol);
    riskMatrix = readMatrixFile("../terrainData/crater/risk_map.txt", Nraw, Ncol);
    planner.initNodeMatrix(elevationMatrix, frictionMatrix, slipMatrix, riskMatrix);
    state = WAITING_GOAL;
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

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
            wRover.position[0] = pose.position[0];
            wRover.position[1] = pose.position[1];
            wRover.position[2] = pose.position[2];
            Eigen::Vector3d euler = pose.orientation.toRotationMatrix().eulerAngles(2, 1, 0);
            wRover.heading = euler[0];
            std::cout<< "Rover at position (" << wRover.position[0] << "," << wRover.position[1] << "," << wRover.position[2] << ") with Heading "
                     << wRover.heading << " rad" << std::endl;
            state = FINDING_PATH;
        }
    }

    if (state == FINDING_PATH)
    {
        trajectory = planner.fastMarching(wRover,goalWaypoint);
        std::cout<< "Trajectory has " << trajectory.size() << " Waypoints" << std::endl;
        for (unsigned int i = 0; i<trajectory.size(); i++)
	    std::cout << "Waypoint " << i << " -> Pos: (" << trajectory[i].position[0] << "," << trajectory[i].position[1] << ") Height: " << trajectory[i].position[2] 
                      << " Heading: " << trajectory[i].heading << std::endl;
        _trajectory.write(trajectory);
        state = END;
    }
}


std::vector< std::vector<double> > Task::readMatrixFile(std::string map_file, double& Nrow, double& Ncol){

    std::cout<< "Map being loaded from: " << map_file << ", correct?" << std::endl;
    std::vector< std::vector<double> > mapMatrix;
    std::string line;
    std::ifstream eFile(map_file.c_str(), std::ios::in);
    Nrow = 0; Ncol = 0;    
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
