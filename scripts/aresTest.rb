# ExoTeR simulation using V-REP

require 'orocos'
require 'readline'

include Orocos
Orocos.initialize

Orocos.conf.load_dir('../../../../bundles/exoter/config/orogen/')

ENV['LANG'] = 'C'
ENV['LC_NUMERIC'] = 'C'

puts "starting"

Orocos.run 'simulation_vrep::Task' => 'simulation_vrep',
           'path_planning::Task' => 'path_planning',
           'locomotion_control::Task' => 'locomotion_control',
           'locomotion_switcher::Task' => 'locomotion_switcher',
           'wheelwalking_control::Task' => 'wheel_walking_control',
           'waypoint_navigation::Task' => 'waypoint_navigation',
           'controldev::JoystickTask'=>'joystick',
           'motion_translator::Task'=>'motion_translator',
           'command_arbiter::Task'=>'command_arbiter',
           'slippage_estimator::Task'=>'slippage_estimator',
           'ptu_control::Task'=>'ptu_control'  do

  # setup locomotion_control
  puts "Setting up locomotion_control"
  locomotion_control = Orocos.name_service.get 'locomotion_control'
  Orocos.conf.apply(locomotion_control, ['default'], :override => true)
  locomotion_control.configure
  puts "done"

  # setup path_planning
  puts "Setting up path_planning: path_planning"
  path_planning = Orocos.name_service.get 'path_planning'
  path_planning.elevationFile = "../terrainData/decos/elevationMap.txt"
  path_planning.costFile = "../terrainData/decos/local_terrainMap.txt"
  path_planning.globalCostFile = "../terrainData/decos/global_terrainMap.txt"
  path_planning.soilsFile = "../terrainData/decos/terrainList.txt"
  path_planning.local_res = 0.0625
  path_planning.crop_local = true
  path_planning.configure
  puts "done"
  # setup exoter wheel_walking_control
  puts "Setting up wheel_walking_control"
  wheel_walking_control = Orocos.name_service.get 'wheel_walking_control'
  Orocos.conf.apply(wheel_walking_control, ['default'], :override => true)
  wheel_walking_control.configure
  puts "done"

  # setup locomotion_switcher
  puts "Setting up locomotion_switcher"
  locomotion_switcher = Orocos.name_service.get 'locomotion_switcher'
  locomotion_switcher.configure
  puts "done"

  # setup waypoint_navigation
  puts "Setting up waypoint_navigation"
  waypoint_navigation = Orocos.name_service.get 'waypoint_navigation'
  Orocos.conf.apply(waypoint_navigation, ['default'], :override => true)
  waypoint_navigation.configure
  puts "done"

  # setup simulation_vrep
  puts "Setting up simulation_vrep"
  simulation_vrep = Orocos.name_service.get 'simulation_vrep'
  simulation_vrep.configure
  puts "done"

  # setup joystick
  puts "Setting up joystick"
  joystick = Orocos.name_service.get 'joystick'
  Orocos.conf.apply(joystick, ['default', 'logitech_gamepad'], :override => true)
  joystick.configure
  puts "done"

  # setup motion_translator
  puts "Setting up motion_translator"
  motion_translator = Orocos.name_service.get 'motion_translator'
  Orocos.conf.apply(motion_translator, ['default'], :override => true)
  motion_translator.configure
  puts "done"

  # setup command_arbitrer
  puts "Setting up command arbiter"
  arbiter = Orocos.name_service.get 'command_arbiter'
  Orocos.conf.apply(arbiter, ['default'], :override => true)
  arbiter.configure
  puts "done"

  # setup slippage_estimator
  puts "Setting up slippage_estimator"
  slippage_estimator = Orocos.name_service.get 'slippage_estimator'
  slippage_estimator.configure
  puts "done"

  # setup ptu_control
  puts "Setting up ptu_control"
  ptu_control = Orocos.name_service.get 'ptu_control'
  Orocos.conf.apply(ptu_control, ['default'], :override => true)
  ptu_control.configure
  puts "done"

  # Log all ports
  Orocos.log_all_ports

  simulation_vrep.pose.connect_to                      path_planning.pose
  simulation_vrep.goalWaypoint.connect_to              path_planning.goalWaypoint
  simulation_vrep.pose.connect_to                      waypoint_navigation.pose
  simulation_vrep.pose.connect_to                      locomotion_switcher.pose
  simulation_vrep.motors_readings.connect_to           locomotion_switcher.motors_readings
  simulation_vrep.motors_readings.connect_to           locomotion_control.joints_readings
  simulation_vrep.joints_readings.connect_to           locomotion_switcher.joints_readings
  simulation_vrep.joints_readings.connect_to           wheel_walking_control.joint_readings
  simulation_vrep.ptu_readings.connect_to              ptu_control.ptu_samples

  #motion_translator.ptu_command.connect_to             ptu_control.ptu_joints_commands

  path_planning.trajectory.connect_to                  simulation_vrep.trajectory
  path_planning.trajectory.connect_to	               waypoint_navigation.trajectory
  path_planning.locomotionMode.connect_to	       locomotion_switcher.locomotionMode
  path_planning.ptu_commands_out.connect_to            ptu_control.ptu_joints_commands

  waypoint_navigation.current_segment.connect_to       locomotion_switcher.current_segment
  waypoint_navigation.currentWaypoint.connect_to       locomotion_switcher.currentWaypoint

  locomotion_control.joints_commands.connect_to        locomotion_switcher.lc_joints_commands

  wheel_walking_control.joint_commands.connect_to      locomotion_switcher.ww_joints_commands

  locomotion_switcher.joints_commands.connect_to       simulation_vrep.joints_commands
  locomotion_switcher.kill_switch.connect_to           wheel_walking_control.kill_switch
  locomotion_switcher.resetDepJoints.connect_to        wheel_walking_control.resetDepJoints
  locomotion_switcher.lc_motion_command.connect_to     locomotion_control.motion_command

  joystick.raw_command.connect_to                       motion_translator.raw_command
  joystick.raw_command.connect_to                       arbiter.raw_command
  motion_translator.motion_command.connect_to           arbiter.joystick_motion_command
  waypoint_navigation.motion_command.connect_to         arbiter.follower_motion_command
  arbiter.motion_command.connect_to                     locomotion_switcher.motion_command
  arbiter.locomotion_mode.connect_to                    locomotion_switcher.locomotionMode_override

  arbiter.motion_command.connect_to                     slippage_estimator.motion_command
  path_planning.locomotionMode.connect_to               slippage_estimator.locomotion_mode
  simulation_vrep.pose.connect_to                       slippage_estimator.pose

  ptu_control.ptu_commands_out.connect_to              simulation_vrep.ptu_commands

  simulation_vrep.start
  sleep 1
  path_planning.start
  waypoint_navigation.start
  locomotion_switcher.start
  wheel_walking_control.start
  locomotion_control.start
  joystick.start
  arbiter.start
  motion_translator.start
  slippage_estimator.start
  ptu_control.start

  Readline::readline("Press ENTER to exit\n")
end
