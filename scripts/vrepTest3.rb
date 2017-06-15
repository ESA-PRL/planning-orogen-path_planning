require 'orocos'
require 'readline'

include Orocos
Orocos.initialize

Orocos.conf.load_dir('../../../../bundles/exoter/config/orogen/')

Orocos.run 'locomotion_control::Task' => 'locomotion_control',
           'locomotion_switcher::Task' => 'locomotion_switcher',
	   'waypoint_navigation::Task' => 'waypoint_navigation',
	   'path_planning::Task' => 'path_planning',
           'wheelwalking_control::Task' => 'wheel_walking_control',
           'simulation_vrep::Task' => 'simulation_vrep' do

  locomotion_control = Orocos.name_service.get 'locomotion_control'
  Orocos.conf.apply(locomotion_control, ['default'], :override => true)
  locomotion_control.configure

  locomotion_switcher = Orocos.name_service.get 'locomotion_switcher'
  locomotion_switcher.configure

  # setup exoter wheel_walking_control
  puts "Setting up wheel_walking_control"
  wheel_walking_control = Orocos.name_service.get 'wheel_walking_control'
  Orocos.conf.apply(wheel_walking_control, ['default'], :override => true)
  wheel_walking_control.configure
  puts "done"

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
  simulation_vrep.pose.connect_to         locomotion_switcher.pose 
  simulation_vrep.goalWaypoint.connect_to path_planning.goalWaypoint

  path_planning.trajectory.connect_to     simulation_vrep.trajectory
  #path_planning.plannerReady.connect_to   simulation_vrep.plannerReady
  path_planning.trajectory.connect_to	  waypoint_navigation.trajectory
  path_planning.trajectory.connect_to     locomotion_switcher.trajectory

  path_planning.locomotionVector.connect_to	  locomotion_switcher.locomotionVector

  locomotion_switcher.joints_commands.connect_to       simulation_vrep.joints_commands
  simulation_vrep.joints_readings.connect_to           wheel_walking_control.joint_readings
  locomotion_switcher.ww_joystick_command.connect_to   wheel_walking_control.joystick_commands
  locomotion_switcher.kill_switch.connect_to           wheel_walking_control.kill_switch
  locomotion_switcher.lc_readings.connect_to           locomotion_control.joints_readings
  locomotion_switcher.lc_motion_command.connect_to     locomotion_control.motion_command
  locomotion_switcher.bema_command.connect_to          locomotion_control.bema_command
  locomotion_switcher.walking_command_front.connect_to locomotion_control.walking_command_front
  locomotion_switcher.walking_command_rear.connect_to  locomotion_control.walking_command_rear

  #simulation_vrep.joints_readings.connect_to           wheel_walking_control.joint_readings
  #simulation_vrep.motors_readings.connect_to           locomotion_control.joints_readings
  simulation_vrep.motors_readings.connect_to           locomotion_switcher.motors_readings
  simulation_vrep.joints_readings.connect_to           locomotion_switcher.joints_readings

  waypoint_navigation.motion_command.connect_to	       locomotion_switcher.joystick_motion_command
  waypoint_navigation.current_segment.connect_to       locomotion_switcher.current_segment
  waypoint_navigation.currentWaypoint.connect_to       locomotion_switcher.currentWaypoint

  locomotion_control.joints_commands.connect_to        locomotion_switcher.lc_commands
  locomotion_control.bema_joints.connect_to            locomotion_switcher.bema_joints

  wheel_walking_control.joint_commands.connect_to      locomotion_switcher.ww_commands 
  
  simulation_vrep.start
  sleep 1
  path_planning.start
  waypoint_navigation.start
  locomotion_switcher.start
  wheel_walking_control.start
  locomotion_control.start

  Readline::readline("Press ENTER to exit\n")

end
