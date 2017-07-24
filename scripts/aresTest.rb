require 'orocos'
require 'readline'

include Orocos
Orocos.initialize

Orocos.conf.load_dir('../../../../bundles/exoter/config/orogen/')

ENV['LANG'] = 'C'
ENV['LC_NUMERIC'] = 'C'

Orocos.run 'path_planning::Task' => 'path_planning',
           'simulation_vrep::Task' => 'simulation_vrep',
           'locomotion_control::Task' => 'locomotion_control',
           'waypoint_navigation::Task' => 'waypoint_navigation', 
           'traversability_explorer::Task' => 'trav',
           "valgrind" => false,
           'output' => nil,
           "wait" => 1000 do

  path_planning = Orocos.name_service.get 'path_planning'
  path_planning.elevationFile = "../terrainData/prl/prl_elevationMap.txt"
  path_planning.costFile = "../terrainData/prl/prl_costMapLander.txt"
  path_planning.globalCostFile = "../terrainData/prl/prl_globalCostMap0.txt"
  path_planning.riskFile = "../terrainData/prl/prl_riskMap.txt"
  path_planning.soilsFile = "../terrainData/prl/soilList.txt"
  path_planning.configure

  locomotion_control = Orocos.name_service.get 'locomotion_control'
  Orocos.conf.apply(locomotion_control, ['default'], :override => true)
  locomotion_control.configure

  simulation_vrep = Orocos.name_service.get 'simulation_vrep'
  simulation_vrep.configure

  waypoint_navigation = Orocos.name_service.get 'waypoint_navigation'
  Orocos.conf.apply(waypoint_navigation, ['default'], :override => true)
  #waypoint_navigation.apply_conf_file("../../../../control/orogen/waypoint_navigation/config/waypoint_navigation::Task.yml", ["exoter"])
  waypoint_navigation.configure

  trav = TaskContext::get 'trav'
  trav.traversability_map_id = "trav_map"

  trav.traversability_map_scalex =  0.03
  trav.traversability_map_scaley =  0.03
  trav.filename = "../terrainData/prl/costmap_january.txt"
  trav.robot_fov_a = 1.0
  trav.robot_fov_b = 1.5
  trav.robot_fov_l = 1.5
  trav.configure
  

  simulation_vrep.pose.connect_to         path_planning.pose  
  simulation_vrep.goalWaypoint.connect_to path_planning.goalWaypoint
  simulation_vrep.pose.connect_to         waypoint_navigation.pose 
  path_planning.trajectory.connect_to     simulation_vrep.trajectory
  path_planning.trajectory.connect_to	  waypoint_navigation.trajectory
  simulation_vrep.pose.connect_to         trav.robot_pose
  simulation_vrep.motors_readings.connect_to           locomotion_control.joints_readings
  waypoint_navigation.motion_command.connect_to	       locomotion_control.motion_command
  locomotion_control.joints_commands.connect_to        simulation_vrep.joints_commands
  waypoint_navigation.current_segment.connect_to       path_planning.current_segment
  #trav.traversability_map.connect_to      path_planning.traversability_map 
  
  simulation_vrep.start
  sleep 1
  #trav.start
  path_planning.start
  waypoint_navigation.start
  locomotion_control.start

  Readline::readline("Press ENTER to exit\n")
end
