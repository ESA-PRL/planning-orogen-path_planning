require 'orocos'
require 'readline'

include Orocos
Orocos.initialize

Orocos.conf.load_dir('../../../../bundles/exoter/config/orogen/')

Orocos.run 'path_planning::Task' => 'path_planning',
           'simulation_vrep::Task' => 'simulation_vrep' do

  path_planning = Orocos.name_service.get 'path_planning'
  path_planning.elevationFile = "../terrainData/prl/prl_elevationMap.txt"
  path_planning.costFile = "../terrainData/prl/prl_costMap.txt"
  path_planning.riskFile = "../terrainData/prl/prl_riskMap.txt"
  path_planning.soilsFile = "../terrainData/prl/soilList.txt"
  path_planning.configure

  simulation_vrep = Orocos.name_service.get 'simulation_vrep'
  simulation_vrep.configure

  

  simulation_vrep.pose.connect_to         path_planning.pose  
  simulation_vrep.goalWaypoint.connect_to path_planning.goalWaypoint
  path_planning.trajectory.connect_to     simulation_vrep.trajectory
  
  simulation_vrep.start
  sleep 1
  path_planning.start

  Readline::readline("Press ENTER to exit\n")

end
