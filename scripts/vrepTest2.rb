require 'orocos'
require 'readline'

include Orocos
Orocos.initialize

Orocos.conf.load_dir('../../../../bundles/exoter/config/orogen/')

Orocos.run 'locomotion_control::Task' => 'locomotion_control',
	   'waypoint_navigation::Task' => 'waypoint_navigation',
	   'path_planning::Task' => 'path_planning',
           'simulation_vrep::Task' => 'simulation_vrep' do

  locomotion_control = Orocos.name_service.get 'locomotion_control'
  Orocos.conf.apply(locomotion_control, ['default'], :override => true)
  locomotion_control.configure

  path_planning = Orocos.name_service.get 'path_planning'
  path_planning.configure

  waypoint_navigation = Orocos.name_service.get 'waypoint_navigation'
  Orocos.conf.apply(waypoint_navigation, ['default'], :override => true)
  #waypoint_navigation.apply_conf_file("../../../../control/orogen/waypoint_navigation/config/waypoint_navigation::Task.yml", ["exoter"])
  waypoint_navigation.configure

  simulation_vrep = Orocos.name_service.get 'simulation_vrep'
  simulation_vrep.configure

  

  simulation_vrep.pose.connect_to         path_planning.pose  
  simulation_vrep.pose.connect_to         waypoint_navigation.pose  
  simulation_vrep.goalWaypoint.connect_to path_planning.goalWaypoint
  path_planning.trajectory.connect_to     simulation_vrep.trajectory
  path_planning.trajectory.connect_to	  waypoint_navigation.trajectory

  simulation_vrep.motors_readings.connect_to           locomotion_control.joints_readings
  waypoint_navigation.motion_command.connect_to	       locomotion_control.motion_command
  locomotion_control.joints_commands.connect_to       simulation_vrep.joints_commands
  
  simulation_vrep.start
  sleep 1
  path_planning.start
  waypoint_navigation.start
  locomotion_control.start

  Readline::readline("Press ENTER to exit\n")

end
