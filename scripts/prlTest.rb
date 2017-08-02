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
           'platform_driver::Task' => 'platform_driver',
           'read_joint_dispatcher::Task' => 'read_joint_dispatcher',
           'command_joint_dispatcher::Task' => 'command_joint_dispatcher',
           'ptu_control::Task' => 'ptu_control',
           'motion_translator::Task' => 'motion_translator',
           'vicon::Task' => 'vicon',
           'controldev::JoystickTask'=>'joystick',
           'motion_translator::Task' => 'motion_translator' do

  # setup exoter ptu_control
    puts "Setting up ptu_control"
    ptu_control = Orocos.name_service.get 'ptu_control'
    Orocos.conf.apply(ptu_control, ['default'], :override => true)
    ptu_control.configure
    puts "done"

  # setup vicon
    puts "Setting up vicon"
    vicon = Orocos.name_service.get 'vicon'
    Orocos.conf.apply(vicon, ['default', 'exoter'], :override => true)
    vicon.configure
    puts "done"

  # setup path_planning
    puts "Setting up path_planning"
    path_planning = Orocos.name_service.get 'path_planning'
    path_planning.elevationFile = "../terrainData/prl/prl_elevationMap.txt"
    path_planning.costFile = "../terrainData/prl/prl_costMapLander.txt"
    path_planning.globalCostFile = "../terrainData/prl/prl_globalCostMap2.txt"
    path_planning.riskFile = "../terrainData/prl/prl_riskMap.txt"
    path_planning.soilsFile = "../terrainData/prl/soilList.txt"
    path_planning.configure
    puts "done"

  # setup platform_driver
    puts "Setting up platform_driver"
    platform_driver = Orocos.name_service.get 'platform_driver'
    Orocos.conf.apply(platform_driver, ['default'], :override => true)
    platform_driver.configure
    puts "done"

    # setup read dispatcher
    puts "Setting up reading joint_dispatcher"
    read_joint_dispatcher = Orocos.name_service.get 'read_joint_dispatcher'
    Orocos.conf.apply(read_joint_dispatcher, ['reading'], :override => true)
    read_joint_dispatcher.configure
    puts "done"

    # setup the commanding dispatcher
    puts "Setting up commanding joint_dispatcher"
    command_joint_dispatcher = Orocos.name_service.get 'command_joint_dispatcher'
    Orocos.conf.apply(command_joint_dispatcher, ['commanding'], :override => true)
    command_joint_dispatcher.configure
    puts "done"

    # setup exoter locomotion_control
    puts "Setting up locomotion_control"
    locomotion_control = Orocos.name_service.get 'locomotion_control'
    Orocos.conf.apply(locomotion_control, ['default'], :override => true)
    locomotion_control.configure
    puts "done"

    puts "Setting up waypoint nav"
    waypoint_navigation = Orocos.name_service.get 'waypoint_navigation'
    Orocos.conf.apply(waypoint_navigation, ['default'], :override => true)
    #waypoint_navigation.apply_conf_file("../../../../control/orogen/waypoint_navigation/config/waypoint_navigation::Task.yml", ["exoter"])
    waypoint_navigation.configure
    puts "done"

    # setup exoter wheel_walking_control
    puts "Setting up wheel_walking_control"
    wheel_walking_control = Orocos.name_service.get 'wheel_walking_control'
    Orocos.conf.apply(wheel_walking_control, ['default'], :override => true)
    wheel_walking_control.configure
    puts "done"

    puts "Setting up locomotion_switcher"
    locomotion_switcher = Orocos.name_service.get 'locomotion_switcher'
    locomotion_switcher.configure
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

  puts "Connecting ports"
  #Orocos.log_all_ports
    
  # Connect ports: ptu_control to command_joint_dispatcher
    ptu_control.ptu_commands_out.connect_to               command_joint_dispatcher.ptu_commands

  # Connecting vicon outputs
    vicon.pose_samples.connect_to                         path_planning.pose
    vicon.pose_samples.connect_to                         waypoint_navigation.pose
    vicon.pose_samples.connect_to                         locomotion_switcher.pose
    #Find a way to give path planning the goal waypoint

  # Connecting path_planning outputs
    path_planning.trajectory.connect_to	                  waypoint_navigation.trajectory
    path_planning.trajectory.connect_to                   locomotion_switcher.trajectory
    path_planning.locomotionVector.connect_to	          locomotion_switcher.locomotionVector
    path_planning.locomotionMode.connect_to	       locomotion_switcher.locomotionMode

  # Connecting waypoint_navigation outputs
    waypoint_navigation.motion_command.connect_to	  locomotion_switcher.joystick_motion_command
    waypoint_navigation.current_segment.connect_to        locomotion_switcher.current_segment
    waypoint_navigation.currentWaypoint.connect_to        locomotion_switcher.currentWaypoint
    waypoint_navigation.current_segment.connect_to        path_planning.current_segment

  # Connecting locomotion_switcher outputs
    locomotion_switcher.joints_commands.connect_to        command_joint_dispatcher.joints_commands         
    locomotion_switcher.ww_joystick_command.connect_to    wheel_walking_control.joystick_commands
    locomotion_switcher.kill_switch.connect_to            wheel_walking_control.kill_switch
    #locomotion_switcher.lc_readings.connect_to            locomotion_control.joints_readings
    locomotion_switcher.lc_motion_command.connect_to      locomotion_control.motion_command
    locomotion_switcher.bema_command.connect_to           locomotion_control.bema_command
    locomotion_switcher.walking_command_front.connect_to  locomotion_control.walking_command_front
    locomotion_switcher.walking_command_rear.connect_to   locomotion_control.walking_command_rear

  # Connecting locomotion_control outputs
    locomotion_control.joints_commands.connect_to         locomotion_switcher.lc_commands
    locomotion_control.bema_joints.connect_to             locomotion_switcher.bema_joints

  # Connecting wheel_walking_control outputs
    wheel_walking_control.joint_commands.connect_to       locomotion_switcher.ww_commands

  # Connecting command_joint_dispatcher outputs
    command_joint_dispatcher.motors_commands.connect_to   platform_driver.joints_commands

  # Connecting platform_driver outputs
    platform_driver.joints_readings.connect_to            read_joint_dispatcher.joints_readings 

  # Connecting read_joint_dispatcher outputs
    read_joint_dispatcher.motors_samples.connect_to       locomotion_control.joints_readings
    read_joint_dispatcher.motors_samples.connect_to       locomotion_switcher.motors_readings
    read_joint_dispatcher.joints_samples.connect_to       wheel_walking_control.joint_readings
    read_joint_dispatcher.joints_samples.connect_to       locomotion_switcher.joints_readings
    read_joint_dispatcher.ptu_samples.connect_to          ptu_control.ptu_samples

    joystick.raw_command.connect_to                      motion_translator.raw_command
    motion_translator.ptu_command.connect_to             ptu_control.ptu_joints_commands

    puts "done"

  # Start the tasks
    ptu_control.start
    vicon.start
    path_planning.start
    waypoint_navigation.start
    locomotion_switcher.start
    locomotion_control.start
    wheel_walking_control.start
    command_joint_dispatcher.start
    platform_driver.start
    read_joint_dispatcher.start
    motion_translator.start
    joystick.start

    Readline::readline("Press ENTER to exit\n")

end
